#define STEP_1_DIR 2
#define STEP_1_STEP 3

#define STEP_2_DIR 4
#define STEP_2_STEP 5

#define STEP_3_DIR 6
#define STEP_3_STEP 7


#define stepsPerRevolution 200


void setup() {
  // Declare pins as output:
  pinMode(STEP_1_STEP, OUTPUT);
  pinMode(STEP_1_DIR, OUTPUT);

  pinMode(STEP_2_STEP, OUTPUT);
  pinMode(STEP_2_DIR, OUTPUT);

  pinMode(STEP_3_STEP, OUTPUT);
  pinMode(STEP_3_DIR, OUTPUT);
}


void loop() {
  // Set the spinning direction clockwise:
  digitalWrite(STEP_1_DIR, HIGH);
  digitalWrite(STEP_2_DIR, HIGH);
  digitalWrite(STEP_3_DIR, HIGH);


  // Spin the stepper motor 5 revolutions fast:
  for (int i = 0; i < 5 * stepsPerRevolution; i++) {
    // These four lines result in 1 step:
    digitalWrite(STEP_1_STEP, HIGH);
    digitalWrite(STEP_2_STEP, HIGH);
    digitalWrite(STEP_3_STEP, HIGH);
    delayMicroseconds(500); 
    digitalWrite(STEP_1_STEP, LOW);
    digitalWrite(STEP_2_STEP, LOW);
    digitalWrite(STEP_3_STEP, LOW);
    delayMicroseconds(500);
  }
  delay(1000);
  
}
