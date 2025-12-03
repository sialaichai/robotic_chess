/*
 * ==================================================================================
 * CHESS ROBOT CONTROLLER (Firmware v2.0)
 * ==================================================================================
 * The "Muscle" of the robot.
 * * RESPONSIBILITIES:
 * 1. Receives commands from Python via Serial (USB).
 * Format: "Base,Shoulder,Elbow,Wrist,Gripper" (in Microseconds).
 * 2. Converts these targets into smooth, slow motion to prevent
 * jerking, swinging, or damaging the gears.
 * 3. Sends a "done" signal back to Python when movement is finished.
 * * TECHNICAL NOTE:
 * We use writeMicroseconds() instead of write() (Degrees).
 * - Degrees (0-180) are integers. A 1-degree step at 30cm reach = ~5mm jump.
 * - Microseconds (500-2500) give us 2000 steps of resolution.
 * This allows for sub-millimeter precision.
 * ==================================================================================
 */

#include <Servo.h>

// --- PIN DEFINITIONS ---
// Ensure these match your physical wiring on the Sensor Shield.
const int PIN_BASE     = 10;
const int PIN_SHOULDER = 8;
const int PIN_ELBOW    = 9;
const int PIN_WRIST    = 12; 
const int PIN_GRIPPER  = 11;

// --- SERVO CALIBRATION (USER CUSTOM) ---
// These values define the physical range of your specific servos.
// MG996R Standard: 500us to 2500us.
// Your Tuning: 0deg = 400us, 180deg = 2400us.
// This shifts the center point slightly to align your specific hardware.
const int MIN_PULSE = 400;
const int MAX_PULSE = 2400;

// --- SPEED & SMOOTHING SETTINGS ---
// This is the "Motion Profile". 
// Total Move Time = MOVE_STEPS * MOVE_DELAY (ms).
// Current: 200 * 15 = 3000ms (3.0 seconds) per move.
// Increase STEPS for smoother motion. Increase DELAY for slower motion.
int MOVE_STEPS = 200;  
int MOVE_DELAY = 15;   

// Servo Objects
Servo base;
Servo shoulder;
Servo elbow;
Servo wrist;
Servo gripper;

// --- CURRENT STATE TRACKING ---
// We must remember where the robot IS to calculate the path to where it goes.
// Initial values are estimates for a safe "Home" pose.
// 1550 = Base Center (approx)
// 1400 = Shoulder Vertical
// 2400 = Elbow Straight
// 400  = Wrist Flat / Gripper Open
float curBase = 1550;     
float curShoulder = 1400; 
float curElbow = 2400;    
float curWrist = 600;     
float curGripper = 900;   

void setup() {
  // Start Serial Communication at 9600 baud (Must match Python)
  Serial.begin(9600);
  
  // Attach Servos with the custom Min/Max pulse limits
  // This prevents the code from accidentally driving the motor into a wall.
  base.attach(PIN_BASE, MIN_PULSE, MAX_PULSE);
  shoulder.attach(PIN_SHOULDER, MIN_PULSE, MAX_PULSE);
  elbow.attach(PIN_ELBOW, MIN_PULSE, MAX_PULSE);
  wrist.attach(PIN_WRIST, MIN_PULSE, MAX_PULSE);
  gripper.attach(PIN_GRIPPER, MIN_PULSE, MAX_PULSE);

  // Move to Initial Safe Pose immediately on startup
  moveServos(curBase, curShoulder, curElbow, curWrist, curGripper); 
  
  // Handshake with Python
  Serial.println("ready");
}

void loop() {
  // Wait for data from Python
  if (Serial.available() > 0) {
    
    // Parse the incoming string "1500,1500,2000,..."
    // parseFloat() reads numeric characters until it hits a comma.
    float tBase = Serial.parseFloat();
    float tShoulder = Serial.parseFloat();
    float tElbow = Serial.parseFloat();
    float tWrist = Serial.parseFloat();
    float tGripper = Serial.parseFloat();

    // Clear any remaining characters (newlines, etc.) from the buffer
    while(Serial.available() && Serial.read() != '\n');

    // Execute the motion profile
    smoothMove(tBase, tShoulder, tElbow, tWrist, tGripper);
    
    // Tell Python we are finished so it can send the next command
    Serial.println("done");
  }
}

/*
 * SMOOTH MOVE ALGORITHM (Linear Interpolation)
 * Instead of jumping instantly from A to B (which causes violent shaking),
 * we slice the movement into 200 tiny steps.
 */
void smoothMove(float b, float s, float e, float w, float g) {
  // 1. Calculate the "Delta" (Distance to travel) for each servo
  // 2. Divide by Steps to get the "Increment per Loop"
  float dB = (b - curBase) / MOVE_STEPS;
  float dS = (s - curShoulder) / MOVE_STEPS;
  float dE = (e - curElbow) / MOVE_STEPS;
  float dW = (w - curWrist) / MOVE_STEPS;
  float dG = (g - curGripper) / MOVE_STEPS;

  // 3. Loop 200 times
  for (int i = 0; i < MOVE_STEPS; i++) {
    curBase += dB;
    curShoulder += dS;
    curElbow += dE;
    curWrist += dW;
    curGripper += dG;

    // Apply the tiny increment
    // Casting to (int) creates the signal, but keeping 'curBase' as float prevents rounding drift.
    base.writeMicroseconds((int)curBase);
    shoulder.writeMicroseconds((int)curShoulder);
    elbow.writeMicroseconds((int)curElbow);
    wrist.writeMicroseconds((int)curWrist);
    gripper.writeMicroseconds((int)curGripper);
    
    // Wait (controls speed)
    delay(MOVE_DELAY); 
  }
  
  // 4. Final Snap
  // Floating point math can have tiny errors (e.g., ending at 1499.9 instead of 1500).
  // We force the final write to the exact target to be sure.
  moveServos(b, s, e, w, g);
}

// Helper to write values instantly (used for setup and final snap)
void moveServos(int b, int s, int e, int w, int g) {
  base.writeMicroseconds(b);
  shoulder.writeMicroseconds(s);
  elbow.writeMicroseconds(e);
  wrist.writeMicroseconds(w);
  gripper.writeMicroseconds(g);
  
  // Update global state
  curBase = b; curShoulder = s; curElbow = e; curWrist = w; curGripper = g;
}
