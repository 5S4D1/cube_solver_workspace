/*
  ESP32 Serial Listener for Rubik’s Cube Solver
  ---------------------------------------------
  - Receives moves from ROS2 via USB serial (e.g. "F", "R2", "U'")
  - Displays them on Serial Monitor
  - (Later you’ll add servo control for each move)
*/

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ; // Wait for serial connection
  }

  Serial.println("✅ ESP32 Rubik's Cube Controller Ready");
  Serial.println("Waiting for moves from ROS2...");
}

void loop() {
  if (Serial.available()) {
    String move = Serial.readStringUntil('\n');
    move.trim();

    if (move.length() > 0) {
      Serial.print("📥 Received move: ");
      Serial.println(move);

      // Later we’ll replace this section with servo logic
      processMove(move);
    }
  }
}

void processMove(String move) {
  if (move == "F") {
    Serial.println("➡️  Rotate Front face clockwise");
  } 
  else if (move == "F'") {
    Serial.println("⬅️  Rotate Front face counter-clockwise");
  } 
  else if (move == "F2") {
    Serial.println("🔁 Rotate Front face 180 degrees");
  } 
  else if (move == "R") {
    Serial.println("➡️  Rotate Right face clockwise");
  } 
  else if (move == "R'") {
    Serial.println("⬅️  Rotate Right face counter-clockwise");
  } 
  else if (move == "R2") {
    Serial.println("🔁 Rotate Right face 180 degrees");
  } 
  else if (move == "U") {
    Serial.println("⬆️  Rotate Up face clockwise");
  } 
  else if (move == "U'") {
    Serial.println("⬇️  Rotate Up face counter-clockwise");
  } 
  else if (move == "U2") {
    Serial.println("🔁 Rotate Up face 180 degrees");
  } 
  else if (move == "L") {
    Serial.println("➡️  Rotate Left face clockwise");
  } 
  else if (move == "L'") {
    Serial.println("⬅️  Rotate Left face counter-clockwise");
  } 
  else if (move == "L2") {
    Serial.println("🔁 Rotate Left face 180 degrees");
  } 
  else if (move == "B") {
    Serial.println("➡️  Rotate Back face clockwise");
  } 
  else if (move == "B'") {
    Serial.println("⬅️  Rotate Back face counter-clockwise");
  } 
  else if (move == "B2") {
    Serial.println("🔁 Rotate Back face 180 degrees");
  } 
  else if (move == "D") {
    Serial.println("⬇️  Rotate Down face clockwise");
  } 
  else if (move == "D'") {
    Serial.println("⬆️  Rotate Down face counter-clockwise");
  } 
  else if (move == "D2") {
    Serial.println("🔁 Rotate Down face 180 degrees");
  } 
  else {
    Serial.print("⚠️ Unknown move: ");
    Serial.println(move);
  }

  delay(1500);
  Serial.println("DONE");
}
