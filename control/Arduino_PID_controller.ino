// These volatile variables keep track of encoder pulses for each motor.
// The "volatile" keyword ensures the compiler doesn't optimize these variables away,
// as they are updated in interrupt service routines (ISRs).
volatile long encoder1Count = 0;
volatile long encoder2Count = 0;

// Define motor control pins for Motor 1 and Motor 2.
// ENA PWM-ENAbled pins for speed control.
// IN's control motor direction.

// Motor 1
#define ENA 8 // YELLOW
#define IN1 9 // BLUE
#define IN2 10 // WHITE

// Motor 2
#define ENB 7 // YELLOW
#define IN3 5 // BLUE
#define IN4 6 // WHITE

// Define the pins connected to the hall sensor outputs (encoders).
// Pins 2 and 3 are interrupt-capable on the Arduino Mega.

#define ENCA1 2  // (Blue) Hall Sensor Encoder A for motor 1
#define ENCB1 3  // (Purple) Hall Sensor Encoder B for motor 1

#define ENCA2 18  // (Blue) Hall Sensor Encoder A for motor 2
#define ENCB2 19  // (Purple) Hall Sensor Encoder B for motor 2


// Define the PID controller gains for each motor:
// Kp = Proportional gain, Ki = Integral gain, Kd = Derivative gain.
float Kp1 = 1, Ki1 = 0.7, Kd1 = 0;
float Kp2 = 1, Ki2 = 0.7, Kd2 = 0;

// Variables for PID control for Motor 1 and Motor 2.
float error1 = 0, lastError1 = 0, integral1 = 0;
float error2 = 0, lastError2 = 0, integral2 = 0;


// Define starting and target positions (in encoder counts) for relative movements.
// posA1/posA2 are the starting positions
// posB1/posB2 define how far to move
long posA1 = 0;           
long posB1 = 10;         

long posA2 = 0;
long posB2 = -20;

// Set the initial target positions to move from 0 to 100 counts.
long target1 = posB1;     
long target2 = posB2;

// Motion state indicates the current direction: 0 = moving forward (to B), 1 = returning (to A).
int motionState = 0;      

// Variables to track timing for PID updates and serial status prints.
unsigned long lastPIDTime = 0;
unsigned long lastPrintTime = 0;

// Acceptable error threshold for reaching the target (in encoder counts).
// The condition to detect when the target is reached now uses a threshold constant
// so that minor overshoots won’t prevent the motion state from switching.

const int ERROR_THRESHOLD = 5;

// Minimum PWM value to overcome static friction or deadband issues in the motor driver.
// In each PID update, if the computed output is nonzero but falls below a minimum value 
// (here defined as 30), the output is bumped up to that minimum. This should overcome 
// the motor’s static friction for small rotations
const int MIN_PWM = 30; 

void setup() {
  // Initialize serial communication at 9600 baud for debugging.
  Serial.begin(9600);

  // Configure motor control pins as outputs.
  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);

  // Configure encoder pins as inputs with internal pull-up resistors enabled.
  // This is useful for the open-collector outputs of hall sensors.
  pinMode(ENCA1, INPUT_PULLUP);
  pinMode(ENCB1, INPUT_PULLUP);
  pinMode(ENCA2, INPUT_PULLUP);
  pinMode(ENCB2, INPUT_PULLUP);

  // Attach interrupt service routines (ISRs) to the encoder channel A pins.
  // These interrupts trigger on a rising edge, ensuring accurate count updates.
  attachInterrupt(digitalPinToInterrupt(ENCA1), encoder1ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCA2), encoder2ISR, RISING);

  // Initialize encoder counts to zero.
  encoder1Count = 0;
  encoder2Count = 0;
}

void loop() {
  // Capture the current time in milliseconds.
  unsigned long now = millis();

  // Run the PID control loops every 50ms.
  if (now - lastPIDTime >= 50) {
    updateMotor1PID();
    updateMotor2PID();
    lastPIDTime = now;
  }

  // Print status (encoder counts, target positions, and motion state) every 500ms.
  if (now - lastPrintTime >= 500) {
    printStatus();
    lastPrintTime = now;
  }

  // Check if both motors have reached their target positions within the ERROR_THRESHOLD.
  if (abs(encoder1Count - target1) < ERROR_THRESHOLD && abs(encoder2Count - target2) < ERROR_THRESHOLD) {
    // Stop both motors.
    setMotor(ENA, IN1, IN2, 0);
    setMotor(ENB, IN3, IN4, 0);
    delay(500);  // Brief delay to ensure motors have fully stopped.

    // Reverse the direction of movement:
    // If the current motion state is 0 (moving forward), set target to return to start.
    // If the state is 1 (returning), set target to move forward.
    if (motionState == 0) {
      target1 = posA1;  // Return to starting position (0)
      target2 = posA2;
      motionState = 1;
    } else {
      target1 = posB1;  // Move forward by the defined count (e.g., 100)
      target2 = posB2;
      motionState = 0;
    }

    // Reset encoder counts to zero to use relative movement for the next cycle.
    encoder1Count = 0;
    encoder2Count = 0;

    // Reset PID integrals and previous errors to prevent windup.
    integral1 = 0;
    integral2 = 0;
    lastError1 = 0;
    lastError2 = 0;
  }
}


// Update PID control for Motor 1.
void updateMotor1PID() {
  // Read the current encoder count.
  long currentPos = encoder1Count;
  // Calculate the error between the target and current position.
  error1 = target1 - currentPos;
  // Integrate the error over time (assuming a 50ms loop, i.e., 0.05 seconds).
  integral1 += error1 * 0.05;
  // Calculate the derivative (rate of change) of the error.
  float derivative = (error1 - lastError1) / 0.05;

  // Compute the PID output using the proportional, integral, and derivative terms.
  float output = Kp1 * error1 + Ki1 * integral1 + Kd1 * derivative;
  // Constrain the output to the valid PWM range (-255 to 255).
  output = constrain(output, -255, 255);

  // Enforce a minimum PWM to overcome static friction (deadband compensation).
  if (abs(output) > 0 && abs(output) < MIN_PWM) {
    output = (output > 0) ? MIN_PWM : -MIN_PWM;
  }

  // Send the computed PWM and direction to the motor.
  setMotor(ENA, IN1, IN2, output);

  // Update last error for the next PID cycle.
  lastError1 = error1;
}

// Update PID control for Motor 2.
void updateMotor2PID() {
  // Read the current encoder count.
  long currentPos = encoder2Count;
  // Calculate the error.
  error2 = target2 - currentPos;
  // Integrate the error over time.
  integral2 += error2 * 0.05;
  // Calculate the derivative of the error.
  float derivative = (error2 - lastError2) / 0.05;

  // Compute the PID output.
  float output = Kp2 * error2 + Ki2 * integral2 + Kd2 * derivative;
  // Constrain the output to the PWM limits.
  output = constrain(output, -255, 255);

  // Enforce a minimum PWM if needed.
  if (abs(output) > 0 && abs(output) < MIN_PWM) {
    output = (output > 0) ? MIN_PWM : -MIN_PWM;
  }

  // Update the motor with the computed value.
  setMotor(ENB, IN3, IN4, output);

  // Update last error for Motor 2.
  lastError2 = error2;
}


// This helper function sets the motor speed and direction based on the given "speed" value.
void setMotor(int en, int in1, int in2, float speed) {
  // Use analogWrite to output PWM based on the absolute value of speed.
  analogWrite(en, abs(speed));

  // Set the motor direction:
  // Positive speed => one direction; negative speed => reverse.
  if (speed > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else if (speed < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  } else { // If speed is 0, stop the motor by setting both direction pins LOW.
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
}


// Interrupt Service Routine (ISR) for Motor 1's encoder.
void encoder1ISR() {
  // Read both encoder channels.
  bool a = digitalRead(ENCA1);
  bool b = digitalRead(ENCB1);
  // Determine direction: if both signals are equal, decrement the count; otherwise, increment.
  encoder1Count += (a == b) ? -1 : 1;
}

// ISR for Motor 2's encoder.
void encoder2ISR() {
  bool a = digitalRead(ENCA2);
  bool b = digitalRead(ENCB2);
  encoder2Count += (a == b) ? -1 : 1;
}


// This function prints motor positions, target positions, and motion state to the serial monitor.
void printStatus() {
  Serial.print("M1 Pos: "); Serial.print(encoder1Count);
  Serial.print(" / Target: "); Serial.print(target1);
  Serial.print(" | M2 Pos: "); Serial.print(encoder2Count);
  Serial.print(" / Target: "); Serial.print(target2);
  Serial.print(" | State: "); Serial.println(motionState);
}
