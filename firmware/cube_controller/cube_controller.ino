#include <Arduino.h>
#include <ESP32Servo.h>
/*
    "F": "Rotate the Front face 90° clockwise",
    "F'": "Rotate the Front face 90° counter-clockwise",
    "F2": "Rotate the Front face 180° (half turn)",
    "B": "Rotate the Back face 90° clockwise",
    "B'": "Rotate the Back face 90° counter-clockwise",
    "B2": "Rotate the Back face 180°",
    "U": "Rotate the Up (top) face 90° clockwise",
    "U'": "Rotate the Up face 90° counter-clockwise",
    "U2": "Rotate the Up face 180°",
    "D": "Rotate the Down (bottom) face 90° clockwise",
    "D'": "Rotate the Down face 90° counter-clockwise",
    "D2": "Rotate the Down face 180°",
    "R": "Rotate the Right face 90° clockwise",
    "R'": "Rotate the Right face 90° counter-clockwise",
    "R2": "Rotate the Right face 180°",
    "L": "Rotate the Left face 90° clockwise",
    "L'": "Rotate the Left face 90° counter-clockwise",
    "L2": "Rotate the Left face 180°",
*/
// ---- Servo Objects ----
Servo baseFront, baseBack, baseLeft, baseRight;
Servo faceFront, faceBack, faceLeft, faceRight;

// ---- Servo Angles ----
int servoNeutral = 90;      // Neutral position for face servos
int servoCW = 180;          // Clockwise rotation
int servoCCW = 0;           // Counter-Clockwise rotation
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
// === release the cube ===
void moveBaseBack() {
  Serial.println("⬅️ Moving base backward (RELEASE)...");
  baseFront.write(servoRelease);
  baseBack.write(servoRelease);
  baseLeft.write(servoRelease);
  baseRight.write(servoRelease);
  delay(800);
}
// === grip the cube ===
void moveBaseForward() {
  Serial.println("➡️ Moving base forward (GRIP)...");
  baseFront.write(servoGrip);
  baseBack.write(servoGrip);
  baseLeft.write(servoGrip);
  baseRight.write(servoGrip);
  delay(800);
}

// ---- Reset Face Servos to Neutral on 90 degree ----
void resetFaceServos() {
  faceFront.write(servoNeutral);
  faceBack.write(servoNeutral);
  faceLeft.write(servoNeutral);
  faceRight.write(servoNeutral);
  delay(400);
}

// ==== Motion Logic ====
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

// === Front Face 90° clockwise Rotations ===
void rotateFrontCW() {
  Serial.println("➡️ Front 90° CW");
  faceFront.write(servoCW);
  delay(800);
  baseFront.write(servoRelease);
  delay(800);
  faceFront.write(servoNeutral);
  delay(600);
  baseFront.write(servoGrip);
  delay(600);
}

// === Front Face 90° counter clockwise Rotations ===
void rotateFrontCCW() {
  Serial.println("⬅️ Front 90° CCW");
  faceFront.write(servoCCW);
  delay(800);
  baseFront.write(servoRelease);
  delay(800);
  faceFront.write(servoNeutral);
  delay(600);
  baseFront.write(servoGrip);
  delay(600);
}

// === Front Face 180° clockwise Rotations ===
void rotateFront180() {
  Serial.println("🔄 Front 90°+90° clockwise");
  
  // First 90° rotation
  rotateFrontCW();

  // Second 90° rotation
  rotateFrontCW();
}

// === Right Face 90° clockwise Rotations ===
void rotateRightCW() {
  Serial.println("➡️ Right 90° CW");
  faceRight.write(servoCW);
  delay(800);
  baseRight.write(servoRelease);
  delay(600);
  faceRight.write(servoNeutral);
  delay(600);
  baseRight.write(servoGrip);
  delay(600);
}

// === Right Face 90° counter clockwise Rotations ===
void rotateRightCCW() {
  Serial.println("⬅️ Right 90° CCW");
  faceRight.write(servoCCW);
  delay(800);
  baseRight.write(servoRelease);
  delay(600);
  faceRight.write(servoNeutral);
  delay(600);
  baseRight.write(servoGrip);
  delay(600);
}
// === Right Face 180° clockwise Rotations ===
void rotateRight180() {
  Serial.println("🔄 Right 180°");
  
  // First 90° rotation
  rotateRightCW();
  
  // Second 90° rotation
  rotateRightCW();
}

// === Left Face 90° clockwise Rotations ===
void rotateLeftCW() {
  Serial.println("➡️ Left 90° CW");
  faceLeft.write(servoCW);
  delay(800);
  baseLeft.write(servoRelease);
  delay(600);
  faceLeft.write(servoNeutral);
  delay(600);
  baseLeft.write(servoGrip);
  delay(800);
}

// === Left Face 90° counter clockwise Rotations ===
void rotateLeftCCW() {
  Serial.println("⬅️ Left 90° CCW");
  faceLeft.write(servoCCW);
  delay(800);
  baseLeft.write(servoRelease);
  delay(600);
  faceLeft.write(servoNeutral);
  delay(600);
  baseLeft.write(servoGrip);
  delay(800);
}

// === Left Face 180° clockwise Rotations ===
void rotateLeft180() {
  Serial.println("🔄 Left 180°");
  
  // First 90° rotation
  rotateLeftCW();
  
  // Second 90° rotation
  rotateLeftCW();
}

// === Back Face 90° clockwise Rotations ===
void rotateBackCW() {
  Serial.println("➡️ Back 90° CW");
  faceBack.write(servoCW);
  delay(800);
  baseBack.write(servoRelease);
  delay(600);
  faceBack.write(servoNeutral);
  delay(400);
  baseBack.write(servoGrip);
  delay(600);
}

// === Back Face 90° counter clockwise Rotations ===
void rotateBackCCW() {
  Serial.println("⬅️ Back 90° CCW");
  faceBack.write(servoCCW);
  delay(800);
  baseBack.write(servoRelease);
  delay(600);
  faceBack.write(servoNeutral);
  delay(400);
  baseBack.write(servoGrip);
  delay(600);
}

// === Back Face 180° clockwise Rotations ===
void rotateBack180() {
  Serial.println("🔄 Back 180°");
  
  // First 90° rotation
  rotateBackCW();
  
  // Second 90° rotation
  rotateBackCCW();
}

// ---- rotate up to front ----
void rotateUptoFront() {
  Serial.println("🔄 Up to front");
  // release right and left side
  baseFront.write(servoRelease);
  baseBack.write(servoRelease);
  delay(600);

  // now rotate the cube
  for (int i = 0; i <= 90; i+=5)
  {
    faceRight.write(servoNeutral - i);
    faceLeft.write(servoNeutral + i);
  }
  
  // Grip right and left side
  baseFront.write(servoGrip);
  baseBack.write(servoGrip);
}

// ---- Rotate Front face to Up face ----
void rotateFronttoUp() {
  Serial.println("🔄 front to up");
  // release Front and Back side
  baseFront.write(servoRelease);
  baseBack.write(servoRelease);
  delay(600);

  // now rotate the cube
  for (int i = 0; i <= 90; i+=5)
  {
    faceRight.write(servoNeutral + i);
    faceLeft.write(servoNeutral - i);
    delay(50);
  }

  // Grip Front and Back side
  baseFront.write(servoGrip);
  baseBack.write(servoGrip);
}

// === Up Face Rotations clockwise ===
void rotateUpCW() {
  Serial.println("⬆️ Up 90° CW");
  // Rotate Up face to Front face
  rotateUptoFront();

  // Rotate Front clockwise
  rotateFrontCW();

  // Rotate Front face to Up face
  rotateFronttoUp();
}

// === Up Face Rotations counter clockwise ===
void rotateUpCCW() {
  Serial.println("⬇️ Up 90° CCW");
  // Rotate Up face to Front face
  rotateUptoFront();

  // Rotate Front counter clockwise face
  rotateFrontCCW();

  // Rotate Front face to Up face
  rotateFronttoUp();
}

// === Up Face Rotations 180° ===
void rotateUp180() {
  Serial.println("🔄 Up 180°");
  // Rotate Up face to Front face
  rotateUptoFront();

  // Rotate Front 180°
  rotateFront180();

  // Rotate Front face to Up face
  rotateFronttoUp();
}

// ---- rotate down to front ----
void rotateDowntoFront() {
  Serial.println("🔄 Down to front");
  // release Front and Back side
  baseFront.write(servoRelease);
  baseBack.write(servoRelease);
  delay(600);

  // now rotate the cube
  for (int i = 0; i <= 90; i+=5)
  {
    faceRight.write(servoNeutral + i);
    faceLeft.write(servoNeutral - i);
    delay(50);
  }

  // Grip Front and Back side
  baseFront.write(servoGrip);
  baseBack.write(servoGrip);
}

// ---- Rotate Front face to Down face ----
void rotateFronttoDown() {
  Serial.println("🔄 front to down");

  // release right and left side
  baseFront.write(servoRelease);
  baseBack.write(servoRelease);
  delay(600);

  // now rotate the cube
  for (int i = 0; i <= 90; i+=5)
  {
    faceRight.write(servoNeutral - i);
    faceLeft.write(servoNeutral + i);
    delay(50);
  }
  // Grip right and left side
  baseFront.write(servoGrip);
  baseBack.write(servoGrip);
}

// === Down Face Rotations ===
void rotateDownCW() {
  Serial.println("⬇️ Down 90° CW");
  // Rotate Front face
  rotateDowntoFront();

  // Rotate Front clockwise
  rotateFrontCW();

  // Rotate Front face to Down face
  rotateFronttoDown();
}

// === Down Face Rotations counter clockwise ===
void rotateDownCCW() {
  Serial.println("⬆️ Down 90° CCW");
  // Rotate Down face to Front face
  rotateDowntoFront();

  // Rotate Front counter clockwise face
  rotateFrontCCW();

  // Rotate Front face to Down face
  rotateFronttoDown();
}

// === Down Face Rotations 180° ===
void rotateDown180() {
  Serial.println("🔄 Down 180°");
  // Rotate Down face to Front face
  rotateDowntoFront();

  // Rotate Front 180°
  rotateFront180();

  // Rotate Front face to Down face
  rotateFronttoDown();
}