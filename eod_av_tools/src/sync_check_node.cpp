// sync_check — mide el desfase real de `header.stamp` de cada topico.
//
// POR QUE NO ALCANZA `ros2 topic delay`
// ------------------------------------
// `ros2 topic delay` es Python y usa una cola RELIABLE. Sobre /lidar_points
// (192000 puntos x 26 B = 5 MB por mensaje a 10 Hz = 50 MB/s) no da abasto: la
// cola crece y lo que reporta como "delay" es su propio atraso. Se vio subir de
// 3,98 s a 4,42 s en 12 s de medicion — 78000 ppm, imposible para un reloj.
//
// Este nodo evita las dos causas:
//   - C++, sin GIL.
//   - QoS BEST_EFFORT con depth 1: si no llega a procesar, DESCARTA en vez de
//     encolar. Un mensaje perdido no sesga la medicion; uno encolado si.
//
// COMO LEE EL TIMESTAMP SIN CONOCER EL TIPO
// -----------------------------------------
// Usa GenericSubscription y lee los 12 primeros bytes del CDR. Todo mensaje
// cuyo primer campo sea un std_msgs/Header los tiene en un lugar fijo:
//
//   offset 0..3  encapsulacion CDR (byte 1: 0x00 = big endian, 0x01 = little)
//   offset 4..7  header.stamp.sec      (int32)
//   offset 8..11 header.stamp.nanosec  (uint32)
//
// Asi funciona igual con PointCloud2, Image, Imu, NavSatFix y con los mensajes
// propios del radar, sin depender de sus paquetes ni recompilar al agregar uno.
// Los topicos que NO empiezan con Header se detectan solos (ver plausible())
// y se reportan aparte en vez de ensuciar la tabla con basura.
//
// QUE REPORTA
// -----------
//   n          mensajes usados
//   media      now - header.stamp, promedio
//   min / max  extremos
//   sd         desvio estandar: la ESTABILIDAD, que es lo que dice si el reloj
//              del sensor esta enganchado o corre libre
//   deriva     pendiente de delay contra tiempo, por minimos cuadrados, en ppm.
//              ~0 = relojes enganchados. Distinto de 0 de forma sostenida = el
//              sensor corre con su propio cristal y se va separando.
//
// USO
//   ros2 run eod_av_tools sync_check      # descubre los topicos solo
//   ros2 run eod_av_tools sync_check --ros-args -p report_period:=15.0
//   ros2 run eod_av_tools sync_check --ros-args -p topics:="['/fix','/lidar_points']"
//
// (Sin barras de continuacion al final de un comentario // : gcc avisa
//  -Wcomment porque la linea siguiente queda comentada sin que se vea.)

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/generic_subscription.hpp>

#include "eod_av_tools/sync_stats.hpp"

using namespace eod_av_tools;  // parseHeaderStamp, Stats, fmt

class SyncCheck : public rclcpp::Node {
public:
  SyncCheck() : Node("sync_check") {
    topics_ = declare_parameter<std::vector<std::string>>(
        "topics", std::vector<std::string>{});
    report_period_ = declare_parameter<double>("report_period", 10.0);
    // Topicos a volcar en HEXADECIMAL en vez de medir. Sirve para leer
    // mensajes cuyo tipo no tiene bindings de Python (`ros2 topic echo` falla
    // con "The message type ... is invalid") o que no empiezan con Header.
    // Caso concreto: hesai_ros_driver/msg/Ptp, que es el unico lugar donde el
    // LiDAR reporta si su PTP esta enganchado.
    raw_topics_ = declare_parameter<std::vector<std::string>>(
        "raw_topics", std::vector<std::string>{});
    discover_period_ = declare_parameter<double>("discover_period", 5.0);

    // BEST_EFFORT + depth 1: es el punto de todo el nodo. Con RELIABLE y una
    // cola profunda, un topico pesado hace que midamos nuestro propio atraso.
    qos_ = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();

    report_timer_ = create_wall_timer(
        std::chrono::duration<double>(report_period_), [this] { report(); });
    // Re-descubrir periodicamente: los nodos pueden arrancar despues que este,
    // y con `topics` fijo tambien sirve para reenganchar si uno reaparece.
    discover_timer_ = create_wall_timer(
        std::chrono::duration<double>(discover_period_), [this] { discover(); });

    RCLCPP_INFO(get_logger(),
                "sync_check: %s, reporte cada %.1f s",
                topics_.empty() ? "descubriendo topicos automaticamente"
                                : "topicos fijados por parametro",
                report_period_);
  }

private:
  bool es_raw(const std::string& t) const {
    return std::find(raw_topics_.begin(), raw_topics_.end(), t) != raw_topics_.end();
  }

  bool wanted(const std::string& t) const {
    if (es_raw(t)) return true;          // los raw se suscriben siempre
    if (!topics_.empty()) {
      return std::find(topics_.begin(), topics_.end(), t) != topics_.end();
    }
    // Auto: todo menos la infraestructura de ROS, que no lleva header util.
    static const std::vector<std::string> skip = {
        "/rosout", "/parameter_events", "/tf", "/tf_static", "/clock",
        // Interaccion de RViz: llevan Header pero no son sensores, y aparecen
        // con n=0 ensuciando la tabla.
        "/clicked_point", "/goal_pose", "/initialpose"};
    return std::find(skip.begin(), skip.end(), t) == skip.end();
  }

  void discover() {
    for (const auto& [name, types] : get_topic_names_and_types()) {
      if (types.empty() || subs_.count(name) || !wanted(name)) continue;
      try {
        // El callback toma shared_ptr NO const a proposito: es la firma que
        // aceptan todas las versiones de create_generic_subscription (la
        // variante con const se agrego despues). Aca no se modifica el mensaje.
        subs_[name] = create_generic_subscription(
            name, types.front(), qos_,
            [this, name](std::shared_ptr<rclcpp::SerializedMessage> msg) {
              onMessage(name, *msg);
            });
        RCLCPP_INFO(get_logger(), "suscrito a %s [%s]", name.c_str(), types.front().c_str());
      } catch (const std::exception& e) {
        RCLCPP_WARN(get_logger(), "no se pudo suscribir a %s: %s", name.c_str(), e.what());
      }
    }
  }

  void onMessage(const std::string& topic, const rclcpp::SerializedMessage& msg) {
    const auto& rcl = msg.get_rcl_serialized_message();
    if (es_raw(topic)) {
      const size_t n = std::min<size_t>(rcl.buffer_length, 64);
      raw_last_[topic].assign(rcl.buffer, rcl.buffer + n);
      raw_len_[topic] = rcl.buffer_length;
      return;
    }
    int64_t sec = 0;
    uint32_t nsec = 0;
    auto& st = stats_[topic];
    if (!parseHeaderStamp(rcl.buffer, rcl.buffer_length, &sec, &nsec)) {
      ++st.rejected;
      return;
    }
    // now() del reloj del nodo (CLOCK_REALTIME, el que disciplina chrony).
    const rclcpp::Time now = get_clock()->now();
    const double wall = static_cast<double>(now.nanoseconds()) * 1e-9;
    const double stamp = static_cast<double>(sec) + static_cast<double>(nsec) * 1e-9;
    st.add(wall - stamp, wall);
  }

  void report() {
    if (stats_.empty() && raw_last_.empty()) {
      RCLCPP_INFO(get_logger(), "todavia no llego ningun mensaje");
      return;
    }
    std::ostringstream o;
    o << "\n"
      << "                              n     media       min       max        sd    deriva\n"
      << "  ------------------------------------------------------------------------------\n";
    for (const auto& [topic, s] : stats_) {
      std::string name = topic;
      if (name.size() > 28) name = "..." + name.substr(name.size() - 25);
      o << "  " << std::left << std::setw(28) << name << std::right;
      if (s.n == 0) {
        o << std::setw(6) << 0 << "   (" << s.rejected
          << " mensajes sin header.stamp plausible)\n";
        continue;
      }
      o << std::setw(6) << s.n
        << std::setw(10) << fmt(s.mean())
        << std::setw(10) << fmt(s.min)
        << std::setw(10) << fmt(s.max)
        << std::setw(10) << fmt(s.stdev())
        << std::setw(9)  << std::fixed << std::setprecision(1) << s.drift_ppm() << "ppm";
      if (s.rejected) o << "  (+" << s.rejected << " descartados)";
      o << "\n";
    }
    o << "\n"
      << "  media  = now - header.stamp = (offset de reloj) + (latencia real del\n"
      << "           sensor: exposicion, transferencia, parseo). No tiene por que\n"
      << "           dar 0, y NO es el desfase de reloj: los dos terminos estan\n"
      << "           sumados y desde aca no se pueden separar. Restar dos medias\n"
      << "           tampoco los separa -- da la diferencia de latencia MAS la de\n"
      << "           reloj. Para separarlos hace falta un evento fisico comun.\n"
      << "  sd     = lo que importa desde aca: ESTABILIDAD. Chico y constante =\n"
      << "           el reloj del sensor sigue al del host.\n"
      << "  deriva = pendiente del delay. ~0 = enganchado. Sostenida y grande =\n"
      << "           el sensor corre con su propio cristal, O el nodo se esta\n"
      << "           atrasando y acumulando backlog (mirá si min tambien sube).\n"
      << "\n"
      << "  TRAMPA: un numero MAS CHICO no es automaticamente mejor. Depende de\n"
      << "  quien puso el sello. Si lo pone el SENSOR (epoca real del dato), la\n"
      << "  media incluye la latencia de entrega y es grande, pero el sello es\n"
      << "  correcto. Si lo pone el HOST al recibir, la media da casi 0 porque\n"
      << "  es now-now: ahi la latencia no desaparecio, se metio adentro del\n"
      << "  dato y dejo de verse. Para un bag, sello del sensor con media\n"
      << "  grande le gana a sello de llegada con media chica.\n";
    for (const auto& [topic, bytes] : raw_last_) {
      o << "\n  --- " << topic << "  (" << raw_len_[topic] << " bytes, primeros "
        << bytes.size() << ") ---\n  ";
      for (size_t i = 0; i < bytes.size(); ++i) {
        char b[4];
        snprintf(b, sizeof(b), "%02x ", bytes[i]);
        o << b;
        if ((i + 1) % 16 == 0 && i + 1 < bytes.size()) o << "\n  ";
      }
      o << "\n";
    }
    RCLCPP_INFO(get_logger(), "%s", o.str().c_str());
  }

  std::vector<std::string> topics_;
  double report_period_ = 10.0;
  double discover_period_ = 5.0;
  rclcpp::QoS qos_{rclcpp::KeepLast(1)};
  std::map<std::string, rclcpp::GenericSubscription::SharedPtr> subs_;
  std::map<std::string, Stats> stats_;
  std::vector<std::string> raw_topics_;
  std::map<std::string, std::vector<uint8_t>> raw_last_;
  std::map<std::string, size_t> raw_len_;
  rclcpp::TimerBase::SharedPtr report_timer_, discover_timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SyncCheck>());
  rclcpp::shutdown();
  return 0;
}
