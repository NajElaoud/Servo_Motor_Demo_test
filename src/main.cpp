// Arduino framework
#include <Arduino.h>

// PCF8574 for I2C LCD
#include <LiquidCrystal_PCF8574.h>

// I2C communication
#include <Wire.h>

// Adafruit PWM library
#include <Adafruit_PWMServoDriver.h>

// Define I2C address - change if required
const int i2c_addr = 0x3F;

// Define LCD object
LiquidCrystal_PCF8574 lcd(i2c_addr);

// PWM parameter definitions
#define MIN_PULSE_WIDTH       650
#define MAX_PULSE_WIDTH       2350
#define FREQUENCY             50

// Define PWM object
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Define potentiometer inputs
int control0 = A0;
int controle1 = A1;

// Define motor oupute on PCA9685 board
int motor0 = 0;
int motor1 = 1;

// Define motor position variables
int mtrDegree0;
int mtrDegree1;

// Define motor previous position variables
int mtrPrevDegree0 = 0;
int mtrPrevDegree1 = 0;

// Variable to determine if display needs updating
boolean updatedisplay = 0;

// Function to move motor to specific position 
void moveMotorDeg(int moveDegree, int motorOut){
  int pulse_wide, pulse_width;

  // Convert to pulse width
  pulse_wide = map(moveDegree, 0, 180, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
  pulse_width = int(float(pulse_wide) / 1000000 * FREQUENCY * 4096);

  // Control motor
  pwm.setPWM(motorOut, 0 ,pulse_width);
}

// Function to convert potentiometer position into servo angle
int getDegree(int controlIn){
  int potVal,srvDegree;
   // Read values from potentiometer
   potVal = analogRead(controlIn);

   // Calculate angle in degrees
   srvDegree = map(potVal, 0, 1023, 0, 180);

   // Return angle in degrees
   return srvDegree;
}

void setup() {
  // Setup PWM controller object
  pwm.begin();
  pwm.setPWMFreq(FREQUENCY);

  // Set display type as 16 char, 2 rows
  lcd.begin(16,2);

  // Turn on LCD Backlight
  lcd.setBacklight(255);

  // Clear the display
  lcd.clear();
  lcd.home();

  // Print on first row of LCD
  lcd.setCursor(0,0);
  lcd.print("Servo 0: 0");

  // Print on second row of LCD
  lcd.setCursor(0,1);
  lcd.print("Servo 1: 0");

  // Start Serial Monitor
  Serial.begin(19200);
}

void loop() {
  // Contorl Servo motor0

  // Get desired position 
  mtrDegree0 = getDegree(control0);

  // Move motor only if control position has changed
  if (mtrDegree0 != mtrPrevDegree0)
  {
    // Move motor
    moveMotorDeg(mtrDegree0,motor0);
    // Update motor moved variable
    updatedisplay = 1;
    // Update previous position
    mtrPrevDegree0 = mtrDegree0;
  }

  // Contorl Servo motor1

  // Get desired position 
  mtrDegree1 = getDegree(controle1);

  // Move motor only if control position has changed
  if (mtrDegree1 != mtrPrevDegree1)
  {
    // Move motor
    moveMotorDeg(mtrDegree1,motor1);
    // Update motor moved variable
    updatedisplay = 1;
    // Update previous position
    mtrPrevDegree1 = mtrDegree1;
  }
  
  // Update display if required
  if (updatedisplay == 1)
  {
    // Clear the display
    lcd.clear();
    lcd.home();

    // Print on first row of LCD
    lcd.setCursor(0,0);
    lcd.print("Servo 0: ");
    lcd.print(mtrDegree0);

    // Print on second row of LCD
    lcd.setCursor(0,1);
    lcd.print("Servo 1: ");
    lcd.print(mtrDegree1);
  }

  // Print to serial monitor
    Serial.print("Motor 0: ");
    Serial.print(mtrDegree0);
    Serial.print("\t");
    Serial.print("Motor 1: ");
    Serial.print(mtrDegree1);
  
  // Reset the motor moved variabla
  updatedisplay = 0;

  // Add short delay
  delay(100);
  
}