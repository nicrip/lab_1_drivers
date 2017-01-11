// Variables for pololu motors
String inData = "";
String inDataSingle = "";
unsigned int servo0, servo1, servo2, servo3, servo4, servo5;
unsigned int count = 0;

// Variables for delayed power to pololu motors
const int motorPowerPin = 24;

// Variables for power distribution and status
const int electronicsPowerPin = 22;
const int switchPowerPin = 25;
const int powerLED = 23;

// Variables for current sensing
const int currentSensor = A0;
int currentValue = 0;

// Variables for voltage sensing
const int voltageSensor = A2;
int voltageValue = 0;

// Variables for humidity sensing
const int humiditySensor = A3;
int humidityValue = 0;

// Variables/libraries for pressure sensing
#include <Wire.h>
#include "Adafruit_Sensor.h"
#include "Adafruit_BMP085_U.h"
Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);

// Variables for pressure and temperature
sensors_event_t event;
float pressure = 0.0;
float temperature = 0.0;

void setup() {
  pinMode(switchPowerPin, OUTPUT);
  digitalWrite(switchPowerPin, LOW);
  pinMode(electronicsPowerPin, OUTPUT);
  digitalWrite(electronicsPowerPin, LOW);
  pinMode(powerLED, OUTPUT);
  digitalWrite(powerLED, LOW);
  pinMode(motorPowerPin, OUTPUT);
  digitalWrite(motorPowerPin, HIGH);
  delay(1000);

  // Setup serial lines for pololu
  Serial.begin(115200);
  Serial1.begin(38400);

  // Switch on electronics power, switch power and status LED
  digitalWrite(switchPowerPin, HIGH);
  digitalWrite(electronicsPowerPin, HIGH);
  digitalWrite(powerLED, HIGH);

  // Setup digital output for motor power
  digitalWrite(motorPowerPin, HIGH);

  // Wait 1 second, then send zero-level values to motor, then wait 30 seconds and switch on motor power
  delay(1000);
  // Push initial zero-level values to each servo
  pushServo(0, 3000);
  pushServo(1, 3000);
  pushServo(2, 3000);
  pushServo(3, 3000);
  pushServo(4, 3000);
  pushServo(5, 3000);
  delay(30000);
  digitalWrite(motorPowerPin, LOW);

  // Initialise the pressure sensor
  bmp.begin();
}

void loop() {
  // Read current value
  currentValue = analogRead(currentSensor);

  // Read voltage value
  voltageValue = analogRead(voltageSensor);

  // Read humidity value
  humidityValue = analogRead(humiditySensor);

  // Read pressure and temperature values
  bmp.getEvent(&event);
  if (event.pressure) {
    pressure = event.pressure;
    bmp.getTemperature(&temperature);
  }

  // FORMAT: "pressure,temperature,current,voltage,humidity\n"
  Serial.print(pressure); Serial.print(','); Serial.print(temperature); Serial.print(','); Serial.print(currentValue); Serial.print(','); Serial.print(voltageValue); Serial.print(','); Serial.println(humidityValue);

  // Control for motors and for shutdown command over serial
  while (Serial.available() > 0)
  {
    char received = Serial.read();
    inData += received;

    if (received == ',') {
      setServo(inDataSingle);
      inDataSingle = "";
      count++;
    } else {
      inDataSingle += received;
    }

    // Process message when new line character is recieved
    if (received == '\n') {
      if (inData == "shutdown\n") {
        // special shutdown message - shutdown motors immediately, wait 30s to shutdown all other power and status LED
        digitalWrite(motorPowerPin, HIGH);
        delay(30000);
        digitalWrite(switchPowerPin, LOW);
        digitalWrite(electronicsPowerPin, LOW);
        delay(1000);
      } else {
        // otherwise we should have received a string of 6 csv values for motors (final value received)
        setServo(inDataSingle);
      }
      inData = ""; // Clear recieved buffer
      inDataSingle = "";
      count = 0;
    }
  }
}

void setServo(String angle) {
  if (count == 0) {
    servo0 = angle.toInt();
    pushServo(0, servo0);
  } else if (count == 1) {
    servo1 = angle.toInt();
    pushServo(1, servo1);
  } else if (count == 2) {
    servo2 = angle.toInt();
    pushServo(2, servo2);
  } else if (count == 3) {
    servo3 = angle.toInt();
    pushServo(3, servo3);
  } else if (count == 4) {
    servo4 = angle.toInt();
    pushServo(4, servo4);
  } else {
    servo5 = angle.toInt();
    pushServo(5, servo5);
  }
}

void pushServo(unsigned int servo, unsigned int angle) {
  //servo is the servo number (typically 0-7)
  //angle is the absolute position from 500 to 5500
  //Send a Pololu Protocol command
  Serial1.write(0x80);               //start byte
  //Serial.print(0x80,HEX);
  Serial1.write(0x01);               //device id
  //Serial.print(0x01,HEX);
  Serial1.write(0x04);               //command number
  //Serial.print(0x04,HEX);
  Serial1.write(servo);              //servo number
  //Serial.print(servo,DEC);
  //Convert the angle data into two 7-bit bytes
  Serial1.write(((angle>>7)&0x3f));  //data1
  //Serial.print(((angle>>7)&0x3f),HEX);
  Serial1.write((angle&0x7f));       //data2
  //Serial.print((angle&0x7f),HEX);
}
