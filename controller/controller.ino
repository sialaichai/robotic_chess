#include <Servo.h>

const int PIN_BASE=10, PIN_SHOULDER=8, PIN_ELBOW=9, PIN_WRIST=12, PIN_GRIPPER=11;
const int MIN_PULSE=400, MAX_PULSE=2400;
int MOVE_STEPS=60;  // Divide the move into 50 small pieces
int MOVE_DELAY=40;  // Wait 30ms between each piece
// Math: 50 * 30ms = 1500ms (1.5 seconds) -> Smooth Motion
Servo base, shoulder, elbow, wrist, gripper;

// --- 1. DEFINE POSITIONS ---

// HOME Position (Original values you provided)
// This is where the robot goes after waking up to wait for commands.
const float HOME_BASE     = 1550;
const float HOME_SHOULDER = 1400;
const float HOME_ELBOW    = 2400;
const float HOME_WRIST    = 600; 
const float HOME_GRIPPER  = 780; // 400 = open to 650

// PARKING Position (Calculated from your degrees)
// Base 90° -> 1400
// Shoulder 70° -> 1178
// Elbow/Wrist 80° -> 1289
// Gripper 34° -> 778
// We initialize the "current" variables to these values so the robot 
// knows it starts in the parked state.
float curBase = 1550;     //1400 
float curShoulder = 1400;  //1178
float curElbow = 1289;     
float curWrist = 1289;     
float curGripper = 780;    

void setup() {
  Serial.begin(9600);
  
  // Attach servos
  base.attach(PIN_BASE, MIN_PULSE, MAX_PULSE);
  shoulder.attach(PIN_SHOULDER, MIN_PULSE, MAX_PULSE);
  elbow.attach(PIN_ELBOW, MIN_PULSE, MAX_PULSE);
  wrist.attach(PIN_WRIST, MIN_PULSE, MAX_PULSE);
  gripper.attach(PIN_GRIPPER, MIN_PULSE, MAX_PULSE);
  
  // 1. LOCK TO PARKING
  // Immediate move to the parking coordinates defined in "cur..." variables above.
  // This ensures the robot holds its folded posture when power is applied.
  moveServos(curBase, curShoulder, curElbow, curWrist, curGripper); 
  //smoothMove(curBase, curShoulder, curElbow, curWrist, curGripper);
  Serial.println("Robot Parked. Initializing...");
  delay(1000); // Short pause to stabilize

  // 2. UNFOLD TO HOME
  // Smoothly move from Parking to Home
  smoothMove(HOME_BASE, HOME_SHOULDER, HOME_ELBOW, HOME_WRIST, HOME_GRIPPER);
  
  Serial.println("ready");
}

void loop() {
  if (Serial.available() > 0) {
    float tBase = Serial.parseFloat();
    float tShoulder = Serial.parseFloat();
    float tElbow = Serial.parseFloat();
    float tWrist = Serial.parseFloat();
    float tGripper = Serial.parseFloat();
    while(Serial.available() && Serial.read()!='\n');

    // Your existing overshoot logic
    //if (tBase > curBase) {
    //   float overshoot = tBase + 20; 
    //   if (overshoot < MAX_PULSE) {
    //     smoothMove(overshoot, tShoulder, tElbow, tWrist, tGripper);
    //     delay(50);
    //   }
    //}
    smoothMove(tBase, tShoulder, tElbow, tWrist, tGripper);
    Serial.println("done");
  }
}

void smoothMove(float b, float s, float e, float w, float g) {
  float dB=(b-curBase)/MOVE_STEPS, dS=(s-curShoulder)/MOVE_STEPS, dE=(e-curElbow)/MOVE_STEPS, dW=(w-curWrist)/MOVE_STEPS, dG=(g-curGripper)/MOVE_STEPS;
  
  for (int i=0; i<MOVE_STEPS; i++) {
    curBase+=dB; curShoulder+=dS; curElbow+=dE; curWrist+=dW; curGripper+=dG;
    base.writeMicroseconds((int)curBase); 
    shoulder.writeMicroseconds((int)curShoulder);
    elbow.writeMicroseconds((int)curElbow); 
    wrist.writeMicroseconds((int)curWrist); 
    gripper.writeMicroseconds((int)curGripper);
    delay(MOVE_DELAY); 
  }
  // Ensure we land exactly on the target integers
  moveServos(b, s, e, w, g);
}

void moveServos(int b, int s, int e, int w, int g) {
  base.writeMicroseconds(b); 
  shoulder.writeMicroseconds(s); 
  elbow.writeMicroseconds(e); 
  wrist.writeMicroseconds(w); 
  gripper.writeMicroseconds(g);
  
  // Update global tracking variables
  curBase=b; curShoulder=s; curElbow=e; curWrist=w; curGripper=g;
}
