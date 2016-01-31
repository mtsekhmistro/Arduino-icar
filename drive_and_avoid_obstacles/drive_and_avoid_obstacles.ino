// include the servo library
#include <Servo.h>

//int pinLB = 6; // Define pin left after 6
//int pinLF = 9; // Define the 9-pin front left
//
//int pinRB = 10;// 10 pin definitions right rear
//int pinRF = 11;// Define the 11-pin front right

// define I1 interface.
int pinLB = 9;
// define I2 interface.
int pinLF = 8; 
// define EA (PWM control) Interface.
int speedpin = 11; 

// define I3 Interface.
int pinRB = 6;
// define I4 Interface.
int pinRF = 7;
// define EB (PWM control) Interface.
int speedpin1 = 10;


int inputPin = A0; // Define pin ultrasonic signal reception
int outputPin = A1;// Define pin ultrasonic signal transmitter
int Fspeedd = 0;   //-Speed 

int Rspeedd = 0;                // Right speed
int Lspeedd = 0;                // Left-speed
int directionn = 0;          // Front Left = 8 after = 2 = 4 Right = 6     

Servo myservo;                    // Set myservo
int delay_time = 250; // settling time after steering servo motors

int Fgo = 8;                      // Forward
int Rgo = 6;                      // Right
int Lgo = 4;                      // Left
int Bgo = 2;                      // Reverse

// Setup.
void setup() {
    Serial.begin (9600);         // Define motor output pin
    pinMode (pinLB, OUTPUT);     // pin   (PWM)         
    pinMode (pinLF, OUTPUT);     // pin 9 (PWM)
    pinMode (pinRB, OUTPUT);     // pin 10 (PWM)
    pinMode (pinRF, OUTPUT);     // pin 11 (PWM)

    pinMode (inputPin, INPUT);   // Define ultrasound input pin
    pinMode (outputPin, OUTPUT); // Define ultrasonic output pin               

    myservo.attach (5);          // Define servo motor output section 5 pin (PWM)
}

// Forward. 
void advance(int a) {  
  digitalWrite (pinRB, LOW);        // The motor (rear right) action
  digitalWrite (pinRF, HIGH);               
  digitalWrite (pinLB, LOW);        // The motor (left rear) action   
  digitalWrite (pinLF, HIGH);
  delay (a * 100);
}

// Turn right (single wheel).
void right(int b) {
  digitalWrite (pinRB, LOW);          // The motor (rear right) action
  digitalWrite (pinRF, HIGH);
  digitalWrite (pinLB, HIGH);
  digitalWrite (pinLF, HIGH);               
  delay (b * 100);
}

// Turn left (single wheel).
void left(int c) {
  digitalWrite (pinRB, HIGH);               
  digitalWrite (pinRF, HIGH);               
  digitalWrite (pinLB, LOW);          // The motor (left rear) action
  digitalWrite (pinLF, HIGH);
  delay (c * 100);
}

/////
void turnR(int d)    // Turn right(wheel)
{
  digitalWrite(pinRB, LOW);    // The motor(rear right) action
  digitalWrite(pinRF, HIGH);
  digitalWrite(pinLB, HIGH);   
  digitalWrite(pinLF, LOW);    // The motor(front left) action
  delay(d * 100);
}

void turnL(int e)    // Turn left(wheel)
{
  digitalWrite(pinRB, HIGH);
  digitalWrite(pinRF, LOW);    // The motor(front right) action
  digitalWrite(pinLB, LOW);    // The motor(left rear) action
  digitalWrite(pinLF, HIGH);
  delay(e * 100);
}

void stopp(int f)    // Stop
{
  digitalWrite(pinRB, HIGH);
  digitalWrite(pinRF, HIGH);
  digitalWrite(pinLB, HIGH);
  digitalWrite(pinLF, HIGH);
  delay(f * 100);
}

void back(int g)    // Check out
{
  digitalWrite(pinRB, HIGH);    // The motor(rear right) action
  digitalWrite(pinRF, LOW);
  digitalWrite(pinLB, HIGH);    // The motor(left rear) action
  digitalWrite(pinLF, LOW);
  delay(g * 100);
}

void detection()    // Measure three angles(0.90.179)
{
  int delay_time = 250;
  // Settling time   // servo motor after turning
  ask_pin_F();    // Read from front
  if (Fspeedd <10)    // If the distance is less than 10 cm in front of
  {
    stopp(1);   // Clear the output data
    back(2);    // Check out 0.2 seconds 
  }
  
  // If the distance is less than 25 cm in front of.
  if (Fspeedd <25)
  {
    stopp(1);       // Clear the output data
    ask_pin_L();    // Read from left
    delay(delay_time);    // Wait for a stable servo motor
    ask_pin_R();          // Read from the right
    delay(delay_time);    // Wait for a stable servo motor

    // If the distance is greater than the right from the left.
    if (Lspeedd> Rspeedd)
    {
      directionn = Rgo;    // Right away
    }

    // If the left is less than or equal to the distance from the right.
    if (Lspeedd <= Rspeedd) {
      directionn = Lgo;    // Turn Left
    }

    // If the distance to the left and right are less than 10 cm distance.
    if (Lspeedd <10 && Rspeedd <10)
    {
      directionn = Bgo;    // To go after
    }
  }
  // Add as front not less than(greater than) 25 cm.
  else {
    directionn = Fgo;      // Move forward
  }
}

// Measure the distance from the front.
void ask_pin_F()
{
  myservo.write(90);
  // Let ultrasonic transmitter low voltage 2 μ s.
  digitalWrite(outputPin, LOW);
  delayMicroseconds(2);
  // Let ultrasonic transmitter high voltage 10 μ s, where at least 10 s.
  digitalWrite (outputPin, HIGH);
  delayMicroseconds(10);
  // Maintain low voltage ultrasonic transmitter.
  digitalWrite(outputPin, LOW);
  
  // Read worse time difference.
  float Fdistance = pulseIn(inputPin, HIGH);
  Fdistance = Fdistance/5.8/10;  // Time to turn to the distance(unit: cm)
  Serial.print("F distance:");  // Output distance(unit: cm)
  Serial.println(Fdistance);    // Display the distance

  // Read into the distance Fspeedd (former speed).
  Fspeedd = Fdistance;
}

// Measure the distance from the left.
void ask_pin_L()
{
  myservo.write(5);
  delay(delay_time);
  // Let ultrasonic transmitter low voltage 2 μ s.
  digitalWrite(outputPin, LOW);
  delayMicroseconds(2);
  // Let ultrasonic transmitter high voltage 10 μ s, where at least 10 μ s.
  digitalWrite (outputPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(outputPin, LOW);    // Maintain low voltage ultrasonic transmitter
  float Ldistance = pulseIn(inputPin, HIGH);    // Read worse time difference
  Ldistance = Ldistance/5.8/10;    // Time to turn to the distance(unit: cm)
  Serial.print("L distance:");    // Output distance(unit: cm)
  Serial.println(Ldistance);    // Display the distance.

  // Read into the distance Lspeedd(left-speed).
  Lspeedd = Ldistance;
}

// Measure the distance from the right.
void ask_pin_R()
{
  myservo.write(177);

  delay(delay_time);
  // Let ultrasonic transmitter low voltage 2 μ s.
  digitalWrite(outputPin, LOW);
  delayMicroseconds(2);
  // Let ultrasonic transmitter high voltage 10 μ s, where at least 10 μ s.
  digitalWrite (outputPin, HIGH);
  delayMicroseconds(10);

  digitalWrite(outputPin, LOW);    // Maintain low voltage ultrasonic transmitter
  float Rdistance = pulseIn(inputPin, HIGH);    // Read worse time difference
  Rdistance = Rdistance/5.8/10;    // Time to turn to the distance(unit: cm)
  Serial.print("R distance:");    // Output distance(unit: cm)
  Serial.println(Rdistance);      // Display the distance

  // Will read into the distance Rspeedd(Right-speed).
  Rspeedd  = Rdistance;           
}

// MAIN LOOP.
void loop()
{
  // Let servo motor position ready to return to the pre-prepared next time measurement.
  myservo.write (90);

  // Measure the angle and direction of judgment to where to move.
  detection();
  
  // If directionn(direction) = 2(reverse)
  if (directionn == 2) {
    back(8);    // Retrogression(car)
    turnL(2);  // Move slightly to the left (to prevent stuck in dead alley).
    Serial.print("Reverse");    // Display direction(backwards)
  }

  // If directionn(direction) = 6(right turn)
  if (directionn == 6) {
    back(1);
    turnR(6);    // Right
    Serial.print("Right");    // Display direction(turn left)
  }
  
  // If directionn(direction) = 4(turn left)
  if (directionn == 4) {
    back(1);
    turnL(6);    // Left
    Serial.print("Left");    // Display direction(turn right)
  }

  // If directionn(direction) = 8(forward)
  if (directionn == 8) {
    advance(1);    // Normal Forward
    Serial.print("Advance");    // Display direction(forward)
    Serial.print("       ");
  }
}

