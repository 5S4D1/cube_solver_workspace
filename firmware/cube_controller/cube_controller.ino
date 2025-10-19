#include <Arduino.h>
#include <ESP32Servo.h>

// ---- Servo Objects ----
Servo baseFront, baseBack, baseLeft, baseRight;
Servo faceFront, faceBack, faceLeft, faceRight;

// ---- Servo Angles ----
int servoNeutral = 90;       // Neutral position for face servos
int servoCW = 180;           // Clockwise rotation
int servoCCW = 0;            // Counter-Clockwise rotation
int servoGrip = 0;          // Forward position (0°) - AUTO GRIPS
int servoRelease = 90;      // Backward position (90°) - AUTO RELEASES

// ---- Function Declarations ----
void executeMove(String move);
void resetFaceServos();
void moveBaseBack();
void moveBaseForward();

// ---- Setup ----
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("✅ ESP32 Cube Controller Ready");

  // Attach base servos
  baseFront.attach(13);
  baseBack.attach(12);
  baseLeft.attach(14);
  baseRight.attach(27);

  // Attach face servos
  faceFront.attach(26);
  faceBack.attach(25);
  faceLeft.attach(33);
  faceRight.attach(32);

  resetFaceServos();
  moveBaseBack();  // Start in release position
  Serial.println("🔵 Waiting for commands...");
}

// ---- Loop ----
void loop() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    if (input.length() == 0) return;

    Serial.print("📥 Received: ");
    Serial.println(input);

    // Handle base position commands
    if (input == "BASE_BACK") {
      moveBaseBack();  // 90° - auto releases
      Serial.println("DONE");
    } 
    else if (input == "BASE_FORWARD") {
      moveBaseForward();  // 0° - auto grips
      Serial.println("DONE");
    }
    // Handle cube moves
    else if (input.startsWith("F") || input.startsWith("R") || 
             input.startsWith("L") || input.startsWith("B") || 
             input.startsWith("U") || input.startsWith("D")) {
      executeMove(input);
      resetFaceServos();
      Serial.println("DONE");  // ACK to ROS2
    } 
    else {
      Serial.println("⚠️ Unknown command: " + input);
      Serial.println("DONE");  // Still send ACK to prevent timeout
    }
  }
}

// ---- Base Movement Functions ----
void moveBaseBack() {
  Serial.println("⬅️ Moving base backward (RELEASE)...");
  baseFront.write(servoRelease);
  baseBack.write(servoRelease);
  baseLeft.write(servoRelease);
  baseRight.write(servoRelease);
  delay(800);
}

void moveBaseForward() {
  Serial.println("➡️ Moving base forward (GRIP)...");
  baseFront.write(servoGrip);
  baseBack.write(servoGrip);
  baseLeft.write(servoGrip);
  baseRight.write(servoGrip);
  delay(800);
}

// ---- Reset Face Servos to Neutral ----
void resetFaceServos() {
  faceFront.write(servoNeutral);
  faceBack.write(servoNeutral);
  faceLeft.write(servoNeutral);
  faceRight.write(servoNeutral);
  delay(400);
}

// ---- Motion Logic ----
void executeMove(String move) {
  if (move == "F") rotateFrontCW();
  else if (move == "F'") rotateFrontCCW();
  else if (move == "F2") rotateFront180();
  else if (move == "R") rotateRightCW();
  else if (move == "R'") rotateRightCCW();
  else if (move == "R2") rotateRight180();
  else if (move == "L") rotateLeftCW();
  else if (move == "L'") rotateLeftCCW();
  else if (move == "L2") rotateLeft180();
  else if (move == "B") rotateBackCW();
  else if (move == "B'") rotateBackCCW();
  else if (move == "B2") rotateBack180();
  else if (move == "U") rotateUpCW();
  else if (move == "U'") rotateUpCCW();
  else if (move == "U2") rotateUp180();
  else if (move == "D") rotateDownCW();
  else if (move == "D'") rotateDownCCW();
  else if (move == "D2") rotateDown180();
  else Serial.println("⚠️ Unknown move: " + move);
}

// ---- Front Face Rotations ----
void rotateFrontCW() {
  Serial.println("➡️ Front 90° CW");
  faceFront.write(servoCW);
  delay(800);
}

void rotateFrontCCW() {
  Serial.println("⬅️ Front 90° CCW");
  faceFront.write(servoCCW);
  delay(800);
}

void rotateFront180() {
  Serial.println("🔄 Front 180°");
  faceFront.write(servoCW);
  delay(800);
  faceFront.write(servoNeutral);
  delay(200);
  faceFront.write(servoCW);
  delay(800);
}

// ---- Right Face Rotations ----
void rotateRightCW() {
  Serial.println("➡️ Right 90° CW");
  faceRight.write(servoCW);
  delay(800);
}

void rotateRightCCW() {
  Serial.println("⬅️ Right 90° CCW");
  faceRight.write(servoCCW);
  delay(800);
}

void rotateRight180() {
  Serial.println("🔄 Right 180°");
  faceRight.write(servoCW);
  delay(800);
  faceRight.write(servoNeutral);
  delay(200);
  faceRight.write(servoCW);
  delay(800);
}

// ---- Left Face Rotations ----
void rotateLeftCW() {
  Serial.println("➡️ Left 90° CW");
  faceLeft.write(servoCW);
  delay(800);
}

void rotateLeftCCW() {
  Serial.println("⬅️ Left 90° CCW");
  faceLeft.write(servoCCW);
  delay(800);
}

void rotateLeft180() {
  Serial.println("🔄 Left 180°");
  faceLeft.write(servoCW);
  delay(800);
  faceLeft.write(servoNeutral);
  delay(200);
  faceLeft.write(servoCW);
  delay(800);
}

// ---- Back Face Rotations ----
void rotateBackCW() {
  Serial.println("➡️ Back 90° CW");
  faceBack.write(servoCW);
  delay(800);
}

void rotateBackCCW() {
  Serial.println("⬅️ Back 90° CCW");
  faceBack.write(servoCCW);
  delay(800);
}

void rotateBack180() {
  Serial.println("🔄 Back 180°");
  faceBack.write(servoCW);
  delay(800);
  faceBack.write(servoNeutral);
  delay(200);
  faceBack.write(servoCW);
  delay(800);
}

// ---- Up Face Rotations (Requires Cube Rotation) ----
void rotateUpCW() {
  Serial.println("⬆️ Up 90° CW");
  
  // Step 1: Release (move back to 90°)
  moveBaseBack();
  delay(300);
  
  // Step 2: Rotate all bases clockwise
  baseFront.write(servoCW);
  baseBack.write(servoCW);
  baseLeft.write(servoCW);
  baseRight.write(servoCW);
  delay(800);
  
  // Step 3: Grip (move forward to 0°)
  moveBaseForward();
  delay(300);
}

void rotateUpCCW() {
  Serial.println("⬇️ Up 90° CCW");
  
  moveBaseBack();
  delay(300);
  
  baseFront.write(servoCCW);
  baseBack.write(servoCCW);
  baseLeft.write(servoCCW);
  baseRight.write(servoCCW);
  delay(800);
  
  moveBaseForward();
  delay(300);
}

void rotateUp180() {
  Serial.println("🔄 Up 180°");
  rotateUpCW();
  delay(200);
  rotateUpCW();
}

// ---- Down Face Rotations ----
void rotateDownCW() {
  Serial.println("⬇️ Down 90° CW");
  
  moveBaseBack();
  delay(300);
  
  baseFront.write(servoCCW);
  baseBack.write(servoCCW);
  baseLeft.write(servoCCW);
  baseRight.write(servoCCW);
  delay(800);
  
  moveBaseForward();
  delay(300);
}

void rotateDownCCW() {
  Serial.println("⬆️ Down 90° CCW");
  
  moveBaseBack();
  delay(300);
  
  baseFront.write(servoCW);
  baseBack.write(servoCW);
  baseLeft.write(servoCW);
  baseRight.write(servoCW);
  delay(800);
  
  moveBaseForward();
  delay(300);
}

void rotateDown180() {
  Serial.println("🔄 Down 180°");
  rotateDownCW();
  delay(200);
  rotateDownCW();
}