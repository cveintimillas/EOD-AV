/* radar_publisher (ROS2 Jazzy)
 *
 * Captures the radar's UDP multicast traffic with libpcap and decodes the
 * RDI v2 protocol (reverse-engineered July 2026, see package README):
 *  - SOME/IP header big-endian, payload little-endian
 *  - service 220, event 2 = far scan / event 4 = near scan
 *  - 27-byte detection records starting at payload offset 52
 *  - each measurement repeated 6x -> deduplicated by MeasurementCounter
 *  - slots past the detection count hold stale data and are never read
 *  - near datagrams (3508 B) arrive IP-fragmented; the BPF filter only
 *    delivers the first fragment, so records beyond it are unreachable
 *
 * Startup parameters (not dynamic): id, iface, port, bpf_extra,
 * pcap_file (offline replay for testing), pcap_loop.
 */

#include <pcap.h>
#include <cmath>
#include <cstring>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include "ars430_ros_publisher/msg/radar_packet.hpp"
#include "ars430_ros_publisher/msg/radar_detection.hpp"
#include "ars430_ros_publisher/msg/radar_object.hpp"
#include "ars430_ros_publisher/msg/radar_object_list.hpp"

// Resolution multipliers, unchanged from the classic ARS430 RDI
static constexpr float RES_RANGE   = 0.004577776f;
static constexpr float RES_VEL     = 0.004577776f;
static constexpr float RES_ANG     = 0.0000958767f;
static constexpr float RES_RCS     = 0.003051851f;
static constexpr float RES_VAR     = 0.000152593f;
static constexpr float RES_ANGVAR  = 0.0000152593f;
static constexpr float RES_SNR     = 0.1f;
static constexpr float RES_VAMBIG  = 0.0030519f;

// v2 layout, offsets relative to the start of the UDP header
static constexpr uint16_t V2_SERVICE_RDI   = 220;
static constexpr uint16_t V2_EVENT_FAR     = 2;
static constexpr uint16_t V2_EVENT_NEAR    = 4;
static constexpr uint32_t V2_LEN_FAR       = 1556;
static constexpr uint32_t V2_LEN_NEAR      = 3500;
static constexpr uint32_t V2_OFF_ALIVE     = 30;
static constexpr uint32_t V2_OFF_TIMESTAMP = 39;
static constexpr uint32_t V2_OFF_MEASCTR   = 43;
static constexpr uint32_t V2_OFF_COUNT     = 52;
static constexpr uint32_t V2_OFF_LIST      = 60;
static constexpr uint32_t V2_RECORD_SIZE   = 27;

// Object list (service 230): the radar's internal tracker output.
// Layout verified against a real capture: 46-byte slots from UDP offset 58
// (payload offset 50), little-endian, positions scaled 0.01 m/count.
// Slot: 0 id | 1 counter | 2-3 x u16 | 4-5 y s16 | 6-7 magic a8 aa |
//       8-38 zeros | 39-40 rcs s16 (tentative) | 41 prob 0..100 |
//       42-43 ? | 44 status | 45 class
static constexpr uint16_t OBJ_SERVICE      = 230;
static constexpr uint32_t OBJ_OFF_LIST     = 58;
static constexpr uint32_t OBJ_SLOT_SIZE    = 46;
static constexpr uint8_t  OBJ_MAGIC0       = 0xa8;
static constexpr uint8_t  OBJ_MAGIC1       = 0xaa;
static constexpr float    OBJ_POS_SCALE    = 0.01f;

static inline uint16_t rdU16BE(const uint8_t* p) { return (uint16_t)((p[0] << 8) | p[1]); }
static inline uint16_t rdU16LE(const uint8_t* p) { return (uint16_t)(p[0] | (p[1] << 8)); }
static inline int16_t  rdS16LE(const uint8_t* p) { return (int16_t)(p[0] | (p[1] << 8)); }
static inline uint32_t rdU32LE(const uint8_t* p) {
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

class RadarPublisherNode : public rclcpp::Node {
public:
  RadarPublisherNode() : Node("radar_publisher") {
    id_        = static_cast<int>(declare_parameter<int64_t>("id", 1));
    iface_     = declare_parameter<std::string>("iface", "enp9s0");
    port_      = static_cast<int>(declare_parameter<int64_t>("port", 40000));
    bpf_extra_ = declare_parameter<std::string>("bpf_extra", "");
    pcap_file_ = declare_parameter<std::string>("pcap_file", "");
    pcap_loop_ = declare_parameter<bool>("pcap_loop", true);
    pcap_realtime_ = declare_parameter<bool>("pcap_realtime", true);

    pub_ = create_publisher<ars430_ros_publisher::msg::RadarPacket>(
        "/unfiltered_radar_packet_" + std::to_string(id_), rclcpp::QoS(50));
    obj_pub_ = create_publisher<ars430_ros_publisher::msg::RadarObjectList>(
        "/radar_objects_raw_" + std::to_string(id_), rclcpp::QoS(50));

    worker_ = std::thread([this] { captureLoop(); });
  }

  ~RadarPublisherNode() override {
    stopping_ = true;
    if (pd_) pcap_breakloop(pd_);
    if (worker_.joinable()) worker_.join();
    if (pd_) pcap_close(pd_);
  }

private:
  void captureLoop() {
    char errbuf[PCAP_ERRBUF_SIZE] = {0};
    const bool offline = !pcap_file_.empty();
    offline_ = offline;

    if (offline) {
      pd_ = pcap_open_offline(pcap_file_.c_str(), errbuf);
      if (!pd_) {
        RCLCPP_ERROR(get_logger(), "pcap_open_offline(%s): %s", pcap_file_.c_str(), errbuf);
        return;
      }
      RCLCPP_INFO(get_logger(), "Replaying capture file %s", pcap_file_.c_str());
    } else {
      pd_ = pcap_open_live(iface_.c_str(), 65535, 1 /*promisc*/, 100 /*ms*/, errbuf);
      if (!pd_) {
        RCLCPP_ERROR(get_logger(),
                     "pcap_open_live(%s): %s\nIf this is a permission error run:\n"
                     "  sudo setcap 'cap_net_raw=pe' <ws>/install/ars430_ros_publisher/lib/ars430_ros_publisher/radar_publisher",
                     iface_.c_str(), errbuf);
        return;
      }
      RCLCPP_INFO(get_logger(), "Capturing live on %s, udp port %d", iface_.c_str(), port_);
    }

    char bpf[256];
    snprintf(bpf, sizeof(bpf), "udp port %d %s", port_, bpf_extra_.c_str());
    struct bpf_program prog;
    if (pcap_compile(pd_, &prog, bpf, 0, PCAP_NETMASK_UNKNOWN) == 0) {
      pcap_setfilter(pd_, &prog);
      pcap_freecode(&prog);
    } else {
      RCLCPP_WARN(get_logger(), "pcap_compile('%s') failed: %s - capturing unfiltered", bpf, pcap_geterr(pd_));
    }

    while (rclcpp::ok() && !stopping_) {
      int n = pcap_dispatch(pd_, 64, &RadarPublisherNode::trampoline,
                            reinterpret_cast<u_char*>(this));
      if (n < 0) break; // error or breakloop
      if (offline && n == 0) { // end of file
        if (!pcap_loop_) break;
        pcap_close(pd_);
        pd_ = pcap_open_offline(pcap_file_.c_str(), errbuf);
        if (!pd_) break;
      }
    }
    RCLCPP_INFO(get_logger(), "Capture loop finished");
  }

  static void trampoline(u_char* user, const struct pcap_pkthdr* h, const u_char* bytes) {
    reinterpret_cast<RadarPublisherNode*>(user)->handleFrame(h, bytes);
  }

  void handleFrame(const struct pcap_pkthdr* h, const u_char* frame) {
    static constexpr uint32_t ETH_HLEN_ = 14;

    // Offline replay: pace packets with their recorded timestamps so RViz
    // shows the capture at real speed instead of as a burst
    if (offline_ && pcap_realtime_) {
      const int64_t ts = (int64_t)h->ts.tv_sec * 1000000 + h->ts.tv_usec;
      if (last_pkt_us_ > 0) {
        const int64_t dt = ts - last_pkt_us_;
        if (dt > 0 && dt < 500000) {
          std::this_thread::sleep_for(std::chrono::microseconds(dt));
        }
      }
      last_pkt_us_ = ts;
    }

    if (h->caplen < ETH_HLEN_ + 20 + 8) return;
    if (frame[12] != 0x08 || frame[13] != 0x00) return; // IPv4 only

    const u_char* ip = frame + ETH_HLEN_;
    const uint32_t ihl = (ip[0] & 0x0F) * 4;
    if (ip[9] != 17) return;                            // UDP only
    if ((rdU16BE(ip + 6) & 0x1FFF) != 0) return;        // skip non-first fragments

    if (h->caplen < ETH_HLEN_ + ihl + 8) return;
    const uint8_t* udp = ip + ihl;
    const uint32_t avail = h->caplen - ETH_HLEN_ - ihl;  // bytes from UDP header onwards
    parsePacket(udp, avail);
  }

  void parsePacket(const uint8_t* p, uint32_t avail) {
    if (avail < 60) return;
    const uint16_t service = rdU16BE(p + 8);
    const uint16_t event   = rdU16BE(p + 10);
    const uint32_t someipLen = ((uint32_t)p[12] << 24) | ((uint32_t)p[13] << 16) |
                               ((uint32_t)p[14] << 8) | p[15];

    if (!headerLogged_) {
      headerLogged_ = true;
      RCLCPP_INFO(get_logger(),
                  "First SOME/IP header: service=%u event=%u len=%u protocol_ver=%u interface_ver=%u",
                  service, event, someipLen, p[20], p[21]);
    }

    if (service == OBJ_SERVICE) {
      parseObjectList(p, avail);
      return;
    }
    if (service != V2_SERVICE_RDI) {
      if (service != 200) {
        RCLCPP_WARN_ONCE(get_logger(), "Unknown SOME/IP service %u - ignoring", service);
      }
      return; // sensor status: v2 layout unknown, ignored
    }

    const bool v2 = (p[29] == 0x14 && p[51] == 0x01 &&
                     (someipLen == V2_LEN_FAR || someipLen == V2_LEN_NEAR));
    if (v2) {
      RCLCPP_INFO_ONCE(get_logger(), "RDI protocol v2 detected - decoding");
    }
    if (!v2) {
      RCLCPP_WARN_ONCE(get_logger(),
                       "RDI packet is not protocol v2 (len=%u). The legacy v1 layout is not "
                       "supported by this ROS2 port - use the Noetic driver for v1 radars.",
                       someipLen);
      return;
    }

    // Deduplicate the 6 cyclic repetitions of each measurement
    const uint32_t mc = rdU32LE(p + V2_OFF_MEASCTR);
    uint32_t& lastMc = (event == V2_EVENT_FAR) ? lastMcFar_ : lastMcNear_;
    if (mc == lastMc) return;
    lastMc = mc;

    uint16_t count = rdU16LE(p + V2_OFF_COUNT);
    if (count == 0) return;

    if (avail < V2_OFF_LIST + V2_RECORD_SIZE) return;
    const uint16_t nVisible = (avail - V2_OFF_LIST) / V2_RECORD_SIZE;
    if (count > nVisible) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "event=%u has %u detections but only %u fit in the first IP fragment - tail lost",
                           event, count, nVisible);
      count = nVisible;
    }

    ars430_ros_publisher::msg::RadarPacket msg;
    msg.header.stamp = now();
    msg.header.frame_id = "radar_fixed";
    msg.event_id = static_cast<uint8_t>(event);
    msg.time_stamp = rdU32LE(p + V2_OFF_TIMESTAMP);
    msg.measurement_counter = mc;
    msg.vambig = rdS16LE(p + 54) * RES_VAMBIG; // payload offset 46, tentative

    msg.detections.reserve(count);
    for (uint16_t i = 0; i < count; i++) {
      const uint8_t* r = p + V2_OFF_LIST + i * V2_RECORD_SIZE;
      ars430_ros_publisher::msg::RadarDetection d;

      const float range = rdU16LE(r + 0) * RES_RANGE;
      const float az0   = rdS16LE(r + 4) * RES_ANG;
      const float az1   = rdS16LE(r + 6) * RES_ANG;
      const uint8_t prob0 = r[16], prob1 = r[17];

      d.range      = range;
      d.vrel_rad   = rdS16LE(r + 2) * RES_VEL;
      d.el_ang     = rdS16LE(r + 8) * RES_ANG;   // 0 on the near scan
      d.el_ang_var = rdU16LE(r + 10) * RES_ANGVAR;
      d.range_var  = rdU16LE(r + 18) * RES_VAR;
      d.az_ang_var = rdU16LE(r + 24) * RES_ANGVAR;
      d.snr        = rdU16LE(r + 20) * RES_SNR;
      d.flags      = r[22]; // semantics not confirmed; published, never filtered here

      if (prob0 >= prob1) {
        d.az_ang = az0;
        d.rcs    = rdS16LE(r + 12) * RES_RCS;
      } else {
        d.az_ang = az1;
        d.rcs    = rdS16LE(r + 14) * RES_RCS;
      }

      d.pos_x = range * std::cos(d.az_ang);
      d.pos_y = -range * std::sin(d.az_ang); // same y-flip convention as the ROS1 driver
      d.pos_z = range * std::sin(d.el_ang);

      msg.detections.push_back(d);
    }

    pub_->publish(msg);
  }

  // Service 230: tracked-object list. The header shares the RDI v2 layout
  // (timestamps/counters at the same offsets); one packet per radar cycle.
  void parseObjectList(const uint8_t* p, uint32_t avail) {
    if (avail < OBJ_OFF_LIST + OBJ_SLOT_SIZE) return;

    const uint32_t mc = rdU32LE(p + V2_OFF_MEASCTR);
    if (mc == lastMcObj_) return;
    lastMcObj_ = mc;

    ars430_ros_publisher::msg::RadarObjectList msg;
    msg.header.stamp = now();
    msg.header.frame_id = "radar_fixed";
    msg.time_stamp = rdU32LE(p + V2_OFF_TIMESTAMP);
    msg.measurement_counter = mc;

    const uint32_t nVisible = (avail - OBJ_OFF_LIST) / OBJ_SLOT_SIZE;
    for (uint32_t i = 0; i < nVisible; i++) {
      const uint8_t* s = p + OBJ_OFF_LIST + i * OBJ_SLOT_SIZE;
      // Magic doubles as the valid-slot test: empty slots are ff ff / zeros
      if (s[6] != OBJ_MAGIC0 || s[7] != OBJ_MAGIC1) continue;

      ars430_ros_publisher::msg::RadarObject o;
      o.id       = s[0];
      o.pos_x    = rdU16LE(s + 2) * OBJ_POS_SCALE;
      o.pos_y    = rdS16LE(s + 4) * OBJ_POS_SCALE;
      o.rcs      = rdS16LE(s + 39) * RES_RCS;
      o.prob     = s[41];
      o.status   = s[44];
      o.class_id = s[45];
      msg.objects.push_back(o);
    }

    RCLCPP_INFO_ONCE(get_logger(), "Object list (service 230) detected - publishing tracked objects");
    obj_pub_->publish(msg);
  }

  int id_ = 1;
  int port_ = 40000;
  std::string iface_, bpf_extra_, pcap_file_;
  bool pcap_loop_ = true;
  bool pcap_realtime_ = true;
  bool offline_ = false;
  int64_t last_pkt_us_ = 0;

  pcap_t* pd_ = nullptr;
  std::thread worker_;
  volatile bool stopping_ = false;
  bool headerLogged_ = false;
  uint32_t lastMcFar_ = 0xFFFFFFFF, lastMcNear_ = 0xFFFFFFFF, lastMcObj_ = 0xFFFFFFFF;

  rclcpp::Publisher<ars430_ros_publisher::msg::RadarPacket>::SharedPtr pub_;
  rclcpp::Publisher<ars430_ros_publisher::msg::RadarObjectList>::SharedPtr obj_pub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RadarPublisherNode>());
  rclcpp::shutdown();
  return 0;
}
