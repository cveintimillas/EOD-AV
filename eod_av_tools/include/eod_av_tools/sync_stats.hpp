// Parseo del header CDR y estadistica de desfase, sin dependencias de ROS.
//
// Vive en un header aparte para que se pueda testear sin levantar un nodo ni
// tener un middleware corriendo: test/test_sync_stats.cpp lo incluye y valida
// el parseo (los dos endianness, buffers cortos, timestamps no plausibles) y la
// regresion de deriva contra rampas sinteticas de pendiente conocida.
#ifndef EOD_AV_TOOLS__SYNC_STATS_HPP_
#define EOD_AV_TOOLS__SYNC_STATS_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iomanip>
#include <sstream>
#include <string>

namespace eod_av_tools {

// Timestamps fuera de esta ventana no son un header.stamp real: o el topico no
// empieza con un std_msgs/Header, o el equipo nunca sincronizo y esta contando
// desde su propio boot. En los dos casos promediarlos no significa nada, asi
// que se cuentan aparte en vez de ensuciar la tabla.
constexpr int64_t kMinPlausibleSec = 1600000000;  // 2020-09
constexpr int64_t kMaxPlausibleSec = 4000000000;  // 2096

inline bool plausible(int64_t sec, uint32_t nsec) {
  return sec >= kMinPlausibleSec && sec <= kMaxPlausibleSec && nsec < 1000000000u;
}

inline uint32_t rd32(const uint8_t* p, bool little_endian) {
  if (little_endian) {
    return  static_cast<uint32_t>(p[0])
         | (static_cast<uint32_t>(p[1]) << 8)
         | (static_cast<uint32_t>(p[2]) << 16)
         | (static_cast<uint32_t>(p[3]) << 24);
  }
  return  static_cast<uint32_t>(p[3])
       | (static_cast<uint32_t>(p[2]) << 8)
       | (static_cast<uint32_t>(p[1]) << 16)
       | (static_cast<uint32_t>(p[0]) << 24);
}

// Lee header.stamp de un mensaje CDR cuyo PRIMER campo sea un std_msgs/Header:
//
//   offset 0..3   encapsulacion CDR (byte 1: 0x00 big endian, 0x01 little)
//   offset 4..7   header.stamp.sec      (int32)
//   offset 8..11  header.stamp.nanosec  (uint32)
//
// Asi funciona igual con PointCloud2, Image, Imu, NavSatFix y los mensajes
// propios del radar, sin depender de sus paquetes ni recompilar al agregar uno.
// Devuelve false si el buffer es corto o el timestamp no es plausible.
inline bool parseHeaderStamp(const uint8_t* buf, size_t len,
                             int64_t* sec, uint32_t* nsec) {
  if (len < 12) return false;
  const bool le = (buf[1] & 0x01) != 0;
  const int32_t s = static_cast<int32_t>(rd32(buf + 4, le));
  const uint32_t n = rd32(buf + 8, le);
  if (!plausible(s, n)) return false;
  *sec = s;
  *nsec = n;
  return true;
}

// Acumuladores en linea: nada crece con el numero de mensajes, asi que el nodo
// puede quedar corriendo horas sin consumir memoria.
struct Stats {
  uint64_t n = 0;
  uint64_t rejected = 0;          // mensajes con timestamp no plausible
  double sum = 0.0, sumsq = 0.0;
  double min = 0.0, max = 0.0;
  // Para la regresion de deriva: t = segundos desde el primer mensaje.
  double st = 0.0, sd = 0.0, stt = 0.0, std_ = 0.0;
  double t0 = 0.0;
  bool first = true;

  void add(double delay, double wall) {
    if (first) { t0 = wall; min = max = delay; first = false; }
    const double t = wall - t0;
    ++n;
    sum += delay;  sumsq += delay * delay;
    min = std::min(min, delay);
    max = std::max(max, delay);
    st += t;  sd += delay;  stt += t * t;  std_ += t * delay;
  }

  double mean() const { return n ? sum / static_cast<double>(n) : 0.0; }

  double stdev() const {
    if (n < 2) return 0.0;
    const double m = mean();
    const double v = sumsq / static_cast<double>(n) - m * m;
    return v > 0.0 ? std::sqrt(v) : 0.0;
  }

  // Pendiente por minimos cuadrados, en ppm (segundos de delay por segundo de
  // reloj). ~0 = los relojes estan enganchados. Sostenida y grande = el sensor
  // corre con su propio cristal y se va separando del host.
  double drift_ppm() const {
    if (n < 3) return 0.0;
    const double nn = static_cast<double>(n);
    const double den = nn * stt - st * st;
    if (std::fabs(den) < 1e-12) return 0.0;
    return ((nn * std_ - st * sd) / den) * 1e6;
  }
};

inline std::string fmt(double seconds) {
  std::ostringstream o;
  const double a = std::fabs(seconds);
  if (a >= 1.0)       o << std::fixed << std::setprecision(3) << seconds << " s";
  else if (a >= 1e-3) o << std::fixed << std::setprecision(2) << seconds * 1e3 << " ms";
  else                o << std::fixed << std::setprecision(1) << seconds * 1e6 << " us";
  return o.str();
}

}  // namespace eod_av_tools

#endif  // EOD_AV_TOOLS__SYNC_STATS_HPP_
