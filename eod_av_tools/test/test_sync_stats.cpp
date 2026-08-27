// Tests de sync_stats.hpp — sin ROS, sin middleware, sin hardware.
//
//   colcon test --packages-select eod_av_tools
//   # o directo:
//   g++ -std=c++17 -Iinclude -o /tmp/t test/test_sync_stats.cpp && /tmp/t
//
// Que se cubre y por que:
//   - Los DOS endianness del CDR. El byte 1 de la encapsulacion los distingue;
//     leerlo al reves da un timestamp absurdo y silencioso.
//   - Buffer corto: un mensaje vacio no puede hacer leer fuera de rango.
//   - Topicos que NO empiezan con Header: tienen que rechazarse, no producir
//     numeros basura que ensucien la tabla.
//   - La regresion de deriva contra rampas de pendiente CONOCIDA. Es el numero
//     que decide "reloj enganchado" vs "reloj libre", asi que si esta mal, todo
//     el diagnostico esta mal.

#include <cassert>
#include <cstdio>
#include <vector>

#include "eod_av_tools/sync_stats.hpp"

using namespace eod_av_tools;

static std::vector<uint8_t> mkcdr(int32_t sec, uint32_t nsec, bool le) {
  std::vector<uint8_t> b(12, 0);
  b[0] = 0x00;
  b[1] = le ? 0x01 : 0x00;
  auto put = [&](size_t off, uint32_t v) {
    if (le) {
      b[off] = v & 0xff; b[off+1] = (v>>8) & 0xff;
      b[off+2] = (v>>16) & 0xff; b[off+3] = (v>>24) & 0xff;
    } else {
      b[off+3] = v & 0xff; b[off+2] = (v>>8) & 0xff;
      b[off+1] = (v>>16) & 0xff; b[off] = (v>>24) & 0xff;
    }
  };
  put(4, static_cast<uint32_t>(sec));
  put(8, nsec);
  return b;
}

int main() {
  int64_t s = 0;
  uint32_t n = 0;

  // Valores reales capturados del Delay_Req de una Triton.
  auto le = mkcdr(1785784430, 719700304, true);
  assert(parseHeaderStamp(le.data(), le.size(), &s, &n));
  assert(s == 1785784430 && n == 719700304u);
  printf("1) CDR little-endian ................ OK\n");

  auto be = mkcdr(1785784430, 719700304, false);
  assert(parseHeaderStamp(be.data(), be.size(), &s, &n));
  assert(s == 1785784430 && n == 719700304u);
  printf("2) CDR big-endian ................... OK\n");

  assert(!parseHeaderStamp(le.data(), 11, &s, &n));
  printf("3) buffer <12 B rechazado ........... OK\n");

  auto junk = mkcdr(42, 5, true);          // topico sin Header al principio
  assert(!parseHeaderStamp(junk.data(), junk.size(), &s, &n));
  printf("4) topico sin Header rechazado ...... OK\n");

  auto bad = mkcdr(1785784430, 1000000000u, true);
  assert(!parseHeaderStamp(bad.data(), bad.size(), &s, &n));
  printf("5) nanosec >= 1e9 rechazado ......... OK\n");

  // Reloj ENGANCHADO: delay constante de 118 ms con +-0,5 ms de ruido, que es
  // lo que se midio en las Triton reales.
  {
    Stats st;
    unsigned seed = 7;
    auto rnd = [&] {
      seed = seed * 1103515245u + 12345u;
      return ((seed >> 16) & 0x7fff) / 32767.0 - 0.5;
    };
    for (int i = 0; i < 1200; ++i) st.add(0.118 + rnd() * 0.001, i / 12.0);
    printf("6) enganchado: media=%s sd=%s deriva=%.1f ppm\n",
           fmt(st.mean()).c_str(), fmt(st.stdev()).c_str(), st.drift_ppm());
    assert(std::fabs(st.mean() - 0.118) < 0.001);
    assert(st.stdev() < 0.001);
    assert(std::fabs(st.drift_ppm()) < 100.0);   // ruido, no deriva
  }

  // Reloj LIBRE: rampa de 50 ppm exactos. La regresion tiene que recuperarla.
  {
    Stats st;
    for (int i = 0; i < 1200; ++i) {
      const double t = i / 12.0;
      st.add(0.118 + 50e-6 * t, t);
    }
    printf("7) libre 50 ppm: deriva medida = %.2f ppm\n", st.drift_ppm());
    assert(std::fabs(st.drift_ppm() - 50.0) < 0.5);
  }

  // Contraprueba de la contraprueba: con pocos datos no se inventa una deriva.
  {
    Stats st;
    st.add(0.1, 0.0);
    st.add(0.1, 1.0);
    assert(st.drift_ppm() == 0.0);
    printf("8) n<3 no inventa deriva ............ OK\n");
  }

  printf("9) fmt: %s | %s | %s | %s\n",
         fmt(4.42).c_str(), fmt(0.118).c_str(),
         fmt(0.0000018).c_str(), fmt(-0.262).c_str());

  printf("\nTODAS LAS PRUEBAS PASARON\n");
  return 0;
}
