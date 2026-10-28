/*
  Trinetra Robot - Arduino Uno firmware

  Receives vision packets from the laptop over USB serial, reads an HC-SR04
  ultrasonic sensor and drives an L298N motor driver.

  Packet (laptop -> Arduino, 115200 baud, one line per frame):
      V,<detected>,<x_offset>,<size>      e.g.  V,1,-20,35
        detected : 0 or 1
        x_offset : -100 (far left) .. 100 (far right)
        size     : 0 .. 100 (target width, % of frame)

  Status (Arduino -> laptop, about 5 per second):
      STATE=APPROACH DIST=42

  Wiring (see README):
      L298N IN1 -> D5   IN2 -> D6    (left motor)
      L298N IN3 -> D9   IN4 -> D10   (right motor)
      HC-SR04 TRIG -> D11, ECHO -> D12
      ENA/ENB jumpers stay fitted on the L298N.

  The constants below are the same as sim/robot_logic.py.
*/

// ---- Pins ----
const uint8_t PIN_IN1 = 5;    // left  motor, forward PWM
const uint8_t PIN_IN2 = 6;    // left  motor, backward PWM
const uint8_t PIN_IN3 = 9;    // right motor, forward PWM
const uint8_t PIN_IN4 = 10;   // right motor, backward PWM
const uint8_t PIN_TRIG = 11;
const uint8_t PIN_ECHO = 12;

// ---- Behaviour constants (keep equal to sim/robot_logic.py) ----
const int OBSTACLE_CM = 15;           // enter AVOID at or below this distance
const int CLEAR_CM = 25;              // leave AVOID at or above this distance
const int ALIGN_TOL = 15;             // |x_offset| allowed before moving forward
const int SLOW_SIZE = 50;             // slow down at or above this size
const int STOP_SIZE = 80;             // stop at or above this size
const unsigned long FAILSAFE_MS = 500;

const float SEARCH_TURN = 0.4;
const float ALIGN_TURN = 0.35;
const float AVOID_TURN = 0.5;
const float FWD_FAST = 0.8;
const float FWD_SLOW = 0.4;

// ---- Motor tuning ----
const int MAX_PWM = 255;
const int MIN_PWM = 90;               // lowest PWM that still turns your motors; tune this

// ---- Timing ----
const unsigned long CONTROL_MS = 50;  // 20 Hz control loop
const unsigned long STATUS_MS = 200;

enum State { ST_SEARCH, ST_ALIGN, ST_APPROACH, ST_STOP, ST_AVOID, ST_FAILSAFE };

struct Packet {
  int detected;
  int xOffset;
  int size;
};

Packet pkt = {0, 0, 0};
bool havePacket = false;
unsigned long lastPacketMs = 0;
bool avoiding = false;
State state = ST_FAILSAFE;
float distCm = 400.0;

char lineBuf[32];
uint8_t lineLen = 0;
unsigned long lastControlMs = 0;
unsigned long lastStatusMs = 0;

const char* stateName(State s) {
  switch (s) {
    case ST_SEARCH:   return "SEARCH";
    case ST_ALIGN:    return "ALIGN";
    case ST_APPROACH: return "APPROACH";
    case ST_STOP:     return "STOP";
    case ST_AVOID:    return "AVOID";
    default:          return "FAILSAFE";
  }
}

// ---------- Packet parsing ----------

// Parse an integer field. Returns false if the text is not a whole number.
bool parseInt(const char* s, int& out) {
  if (*s == '\0') return false;
  char* end;
  long v = strtol(s, &end, 10);
  if (*end != '\0') return false;
  out = (int)v;
  return true;
}

// Parse "V,<detected>,<x_offset>,<size>". Returns false for invalid lines.
bool parsePacket(char* line, Packet& out) {
  char* f[4];
  uint8_t n = 0;
  char* p = line;
  f[n++] = p;
  while (*p) {
    if (*p == ',') {
      *p = '\0';
      if (n >= 4) return false;      // too many fields
      f[n++] = p + 1;
    }
    p++;
  }
  if (n != 4) return false;
  if (strcmp(f[0], "V") != 0) return false;
  int d, x, s;
  if (!parseInt(f[1], d) || !parseInt(f[2], x) || !parseInt(f[3], s)) return false;
  if (d != 0 && d != 1) return false;
  out.detected = d;
  out.xOffset = constrain(x, -100, 100);
  out.size = constrain(s, 0, 100);
  return true;
}

// Read serial bytes; handle complete lines. Called every loop().
void readSerial(unsigned long now) {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (lineLen > 0) {
        lineBuf[lineLen] = '\0';
        Packet p;
        if (parsePacket(lineBuf, p)) {   // garbage lines are ignored
          pkt = p;
          havePacket = true;
          lastPacketMs = now;
        }
        lineLen = 0;
      }
    } else if (lineLen < sizeof(lineBuf) - 1) {
      lineBuf[lineLen++] = c;
    } else {
      lineLen = 0;                        // line too long: discard
    }
  }
}

// ---------- Sensor ----------

float readDistanceCm() {
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);
  unsigned long us = pulseIn(PIN_ECHO, HIGH, 25000UL);   // 25 ms is about 4 m
  if (us == 0) return 400.0;                             // no echo: treat as far
  return us * 0.0343 / 2.0;
}

// ---------- Motors ----------

int toPwm(float v) {
  float a = fabs(v);
  if (a < 0.01) return 0;
  int pwm = (int)(a * MAX_PWM + 0.5);
  if (pwm < MIN_PWM) pwm = MIN_PWM;
  if (pwm > MAX_PWM) pwm = MAX_PWM;
  return pwm;
}

// v in -1.0 .. 1.0. Positive = forward.
void driveSide(uint8_t pinFwd, uint8_t pinBack, float v) {
  int pwm = toPwm(v);
  if (v > 0) {
    analogWrite(pinFwd, pwm);
    analogWrite(pinBack, 0);
  } else if (v < 0) {
    analogWrite(pinFwd, 0);
    analogWrite(pinBack, pwm);
  } else {
    analogWrite(pinFwd, 0);
    analogWrite(pinBack, 0);
  }
}

// left > 0 and right < 0 turns clockwise (right). Reverse for left.
void drive(float left, float right) {
  driveSide(PIN_IN1, PIN_IN2, left);
  driveSide(PIN_IN3, PIN_IN4, right);
}

// ---------- Decision logic (same as sim/robot_logic.py) ----------

void controlStep(unsigned long now) {
  float left = 0, right = 0;

  // Failsafe: no recent packet -> stop.
  if (!havePacket || now - lastPacketMs > FAILSAFE_MS) {
    state = ST_FAILSAFE;
    drive(0, 0);
    return;
  }

  // Obstacle safety has priority over vision (with hysteresis).
  if (distCm <= OBSTACLE_CM) avoiding = true;
  else if (avoiding && distCm >= CLEAR_CM) avoiding = false;

  if (avoiding) {
    state = ST_AVOID;
    left = AVOID_TURN;
    right = -AVOID_TURN;
  } else if (!pkt.detected) {
    state = ST_SEARCH;
    left = SEARCH_TURN;
    right = -SEARCH_TURN;
  } else if (abs(pkt.xOffset) > ALIGN_TOL) {
    state = ST_ALIGN;
    if (pkt.xOffset < 0) { left = -ALIGN_TURN; right = ALIGN_TURN; }   // target on the left
    else                 { left = ALIGN_TURN;  right = -ALIGN_TURN; }
  } else if (pkt.size >= STOP_SIZE) {
    state = ST_STOP;
  } else {
    state = ST_APPROACH;
    float speed = (pkt.size >= SLOW_SIZE) ? FWD_SLOW : FWD_FAST;
    float k = pkt.xOffset / 200.0;
    left = speed * (1 + k);
    right = speed * (1 - k);
  }
  drive(left, right);
}

// ---------- Arduino entry points ----------

void setup() {
  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);
  pinMode(PIN_IN3, OUTPUT);
  pinMode(PIN_IN4, OUTPUT);
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  drive(0, 0);
  Serial.begin(115200);
  Serial.println("TRINETRA READY");
}

void loop() {
  unsigned long now = millis();
  readSerial(now);

  if (now - lastControlMs >= CONTROL_MS) {
    lastControlMs = now;
    distCm = readDistanceCm();
    controlStep(now);
  }

  if (now - lastStatusMs >= STATUS_MS) {
    lastStatusMs = now;
    Serial.print("STATE=");
    Serial.print(stateName(state));
    Serial.print(" DIST=");
    Serial.println((int)distCm);
  }
}
