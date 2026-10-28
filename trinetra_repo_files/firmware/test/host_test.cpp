// Host-side test: runs trinetra_robot.ino on a PC with fake Arduino functions
// and checks the 13 README scenarios.   Build and run:
//     g++ -std=c++17 -o host_test firmware/test/host_test.cpp && ./host_test
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <cstdint>
#include <string>
#include <deque>

// ---- minimal Arduino stubs ----
#define LOW 0
#define HIGH 1
#define OUTPUT 1
#define INPUT 0
static unsigned long g_ms = 0;
static unsigned long g_echo_us = 0;          // what pulseIn returns
static int g_pwm[20];
static std::deque<char> g_rx;
static std::string g_tx;
unsigned long millis() { return g_ms; }
void pinMode(uint8_t, uint8_t) {}
void digitalWrite(uint8_t, uint8_t) {}
void delayMicroseconds(unsigned) {}
unsigned long pulseIn(uint8_t, uint8_t, unsigned long) { return g_echo_us; }
void analogWrite(uint8_t pin, int v) { g_pwm[pin] = v; }
template <class T> T constrain(T x, T a, T b) { return x < a ? a : (x > b ? b : x); }
struct SerialStub {
  void begin(long) {}
  int available() { return (int)g_rx.size(); }
  int read() { int c = g_rx.front(); g_rx.pop_front(); return c; }
  void print(const char* s) { g_tx += s; }
  void print(int v) { g_tx += std::to_string(v); }
  void println(const char* s) { g_tx += s; g_tx += "\n"; }
  void println(int v) { g_tx += std::to_string(v) + "\n"; }
} Serial;

#include "../trinetra_robot/trinetra_robot.ino"

// ---- test helpers ----
static void send(const char* s) { for (const char* p = s; *p; p++) g_rx.push_back(*p); g_rx.push_back('\n'); }
static void setDist(double cm) { g_echo_us = cm >= 400 ? 0 : (unsigned long)(cm * 2 / 0.0343); }
static void run(unsigned long ms) { for (unsigned long t = 0; t < ms; t += 10) { g_ms += 10; loop(); } }
static void reset() { memset(g_pwm, 0, sizeof g_pwm); g_rx.clear(); g_tx.clear(); g_ms += 5000;
                      havePacket = false; avoiding = false; lineLen = 0; setDist(400); }
static float L() { return (g_pwm[PIN_IN1] - g_pwm[PIN_IN2]) / 255.0f; }
static float R() { return (g_pwm[PIN_IN3] - g_pwm[PIN_IN4]) / 255.0f; }
static int fails = 0, total = 0;
static void check(int n, const char* name, bool ok) { total++; if (!ok) fails++; printf("  %2d. %s  %s\n", n, ok ? "PASS" : "FAIL", name); }

int main() {
  setup();
  printf("Firmware scenarios (host build)\n");
  reset(); send("V,0,0,0"); run(100);
  check(1, "V,0,0,0 -> SEARCH rotates", state == ST_SEARCH && L() > 0 && R() < 0);
  reset(); send("V,1,-60,20"); run(100);
  check(2, "V,1,-60,20 -> ALIGN turns left", state == ST_ALIGN && L() < 0 && R() > 0);
  reset(); send("V,1,60,20"); run(100);
  check(3, "V,1,60,20 -> ALIGN turns right", state == ST_ALIGN && L() > 0 && R() < 0);
  reset(); send("V,1,0,20"); run(100); float f4 = L();
  check(4, "V,1,0,20 -> APPROACH forward", state == ST_APPROACH && L() > 0 && R() > 0);
  reset(); send("V,1,0,60"); run(100);
  check(5, "V,1,0,60 -> APPROACH slower", state == ST_APPROACH && L() > 0 && L() < f4);
  reset(); send("V,1,0,90"); run(100);
  check(6, "V,1,0,90 -> STOP", state == ST_STOP && L() == 0 && R() == 0);
  reset(); send("V,1,0,20"); run(100); send("V,0,0,0"); run(100);
  check(7, "target lost -> SEARCH", state == ST_SEARCH);
  reset(); setDist(15); send("V,1,0,20"); run(100);
  check(8, "obstacle 15 cm -> AVOID", state == ST_AVOID && !(L() > 0 && R() > 0));
  reset(); setDist(100); send("V,1,0,20"); run(100);
  check(9, "obstacle 100 cm -> APPROACH", state == ST_APPROACH);
  reset(); setDist(15); send("V,0,0,0"); run(100);
  check(10, "AVOID overrides SEARCH", state == ST_AVOID);
  reset(); send("V,1,0,20"); run(100); run(1000);
  check(11, "no input 1 s -> failsafe stop", state == ST_FAILSAFE && L() == 0 && R() == 0);
  reset(); send("V,0,0,0"); run(100); send("garbage"); send("V,1,2"); send("V,1,x,3"); send("V,2,0,0"); run(100);
  check(12, "garbage ignored", state == ST_SEARCH);
  reset(); setDist(15); send("V,1,0,20"); run(100); setDist(400); send("V,1,0,20"); run(100);
  check(13, "obstacle removed -> APPROACH", state == ST_APPROACH);
  reset(); send("V,0,0,0"); run(300);
  check(14, "status line printed", g_tx.find("STATE=SEARCH DIST=400") != std::string::npos);
  printf("%d/%d passed\n", total - fails, total);
  return fails ? 1 : 0;
}
