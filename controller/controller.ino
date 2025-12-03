#include <Servo.h>

// --- PIN DEFINITIONS ---
const int PIN_BASE     = 10;
const int PIN_SHOULDER = 8;
const int PIN_ELBOW    = 9;
const int PIN_WRIST    = 12; 
const int PIN_GRIPPER  = 11;

// --- SERVO CALIBRATION (USER CUSTOM) ---
// 0deg = 400us, 90deg = 1400us, 180deg = 2400us
const int MIN_PULSE = 400;
const int MAX_PULSE = 2400;

// --- SPEED SETTINGS ---
int MOVE_STEPS = 200;  // High resolution for smooth moves
int MOVE_DELAY = 15;   // Slow speed

Servo base;
Servo shoulder;
Servo elbow;
Servo wrist;
Servo gripper;

// GLOBAL VARIABLES (In Microseconds)
// Startup: Base 95deg, Shoulder 90, Elbow 180, Wrist 0, Gripper Open
// Calc: 400 + (Deg/180 * 2000)
float curBase = 1550;     // 95 deg approx 1456
float curShoulder = 1400; // 90 deg (Vertical)
float curElbow = 2400;    // 180 deg (Straight)
float curWrist = 800;     // 0 deg was 400
float curGripper = 400;   // 0 deg (Open)

void setup() {
  Serial.begin(9600);
  
  // Attach with custom range
  base.attach(PIN_BASE, MIN_PULSE, MAX_PULSE);
  shoulder.attach(PIN_SHOULDER, MIN_PULSE, MAX_PULSE);
  elbow.attach(PIN_ELBOW, MIN_PULSE, MAX_PULSE);
  wrist.attach(PIN_WRIST, MIN_PULSE, MAX_PULSE);
  gripper.attach(PIN_GRIPPER, MIN_PULSE, MAX_PULSE);

  // Initial Pose
  moveServos(curBase, curShoulder, curElbow, curWrist, curGripper); 
  Serial.println("ready");
}

void loop() {
  if (Serial.available() > 0) {
    // Protocol: "Base,Shld,Elb,Wrist,Grip" in MICROSECONDS
    float tBase = Serial.parseFloat();
    float tShoulder = Serial.parseFloat();
    float tElbow = Serial.parseFloat();
    float tWrist = Serial.parseFloat();
    float tGripper = Serial.parseFloat();

    while(Serial.available() && Serial.read() != '\n');

    smoothMove(tBase, tShoulder, tElbow, tWrist, tGripper);
    Serial.println("done");
  }
}

void smoothMove(float b, float s, float e, float w, float g) {
  float dB = (b - curBase) / MOVE_STEPS;
  float dS = (s - curShoulder) / MOVE_STEPS;
  float dE = (e - curElbow) / MOVE_STEPS;
  float dW = (w - curWrist) / MOVE_STEPS;
  float dG = (g - curGripper) / MOVE_STEPS;

  for (int i = 0; i < MOVE_STEPS; i++) {
    curBase += dB;
    curShoulder += dS;
    curElbow += dE;
    curWrist += dW;
    curGripper += dG;

    base.writeMicroseconds((int)curBase);
    shoulder.writeMicroseconds((int)curShoulder);
    elbow.writeMicroseconds((int)curElbow);
    wrist.writeMicroseconds((int)curWrist);
    gripper.writeMicroseconds((int)curGripper);
    
    delay(MOVE_DELAY); 
  }
  // Snap to final
  moveServos(b, s, e, w, g);
}

void moveServos(int b, int s, int e, int w, int g) {
  base.writeMicroseconds(b);
  shoulder.writeMicroseconds(s);
  elbow.writeMicroseconds(e);
  wrist.writeMicroseconds(w);
  gripper.writeMicroseconds(g);
  
  curBase = b; curShoulder = s; curElbow = e; curWrist = w; curGripper = g;
}
