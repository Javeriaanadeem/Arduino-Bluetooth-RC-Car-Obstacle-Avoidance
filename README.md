Arduino Bluetooth RC Car with Obstacle Avoidance

This project is an Arduino-based Bluetooth-controlled RC car that can detect obstacles using a single ultrasonic sensor and automatically avoid collisions.

Components Details

 Arduino UNO – The main microcontroller that runs the code.

 HC-05 Bluetooth Module – Enables wireless control from a mobile app.

 L298N Motor Driver – Controls the DC motors.

 4 DC Motors with Wheels – Drives the car.

 Ultrasonic Sensor (HC-SR04) – Detects obstacles in front of the car.

 2 x 18650 Rechargeable Batteries – Powers the motors and Arduino.

 Car Chassis – The body/frame to mount components.

 Jumper Wires – For making all electrical connections.

Steps to Build

Mount the Motors
Attach all four motors to the chassis using screws or hot glue.

Wire the Motors to L298N

Left motors → OUT1 and OUT2

Right motors → OUT3 and OUT4

Connect Power

Battery positive → L298N 12V

Battery ground → L298N GND and Arduino GND

L298N 5V → Arduino 5V (to power Arduino from motor driver)

Connect Control Pins
ENA (Pin 9), IN1 (Pin 2), IN2 (Pin 3), IN3 (Pin 4), IN4 (Pin 5), ENB (Pin 6) - Arduino digital pins

Ultrasonic Sensor Wiring

VCC → 5V

GND → GND

Trig → Arduino pin 10

Echo → Arduino pin 11

Bluetooth Module Wiring

VCC → 5V

GND → GND

TX → Arduino pin 7
RX → Arduino pin 8

Install Bluetooth RC Controller App
Download a Bluetooth RC car controller app from the Play Store and pair it with your HC-05.

Circuit Diagram
<img width="1536" height="1024" alt="rc car circuit diagram" src="https://github.com/user-attachments/assets/1328c931-a8be-4a20-b112-f393fcac1151" />

How It Works

The HC-05 module receives commands from the mobile app (Forward, Back, Left, Right, Stop).

The Arduino reads the ultrasonic sensor before moving forward. If the distance is less than 30cm, it automatically stops or avoids the obstacle.

The L298N motor driver controls the motors based on the Arduino's signals.

Code
```
#include <SoftwareSerial.h>
SoftwareSerial BT(7, 8);  // HC-05 TX to 7, RX to 8

#define ENA 9
#define ENB 6
#define IN1 2
#define IN2 3
#define IN3 4
#define IN4 5
#define FRONT_TRIG_PIN 10
#define FRONT_ECHO_PIN 11
#define DISTANCE_THRESHOLD 20

void setup() {
    BT.begin(9600); 
    Serial.begin(9600);
    
    pinMode(ENA, OUTPUT);
    pinMode(ENB, OUTPUT);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);

    pinMode(FRONT_TRIG_PIN, OUTPUT);
    pinMode(FRONT_ECHO_PIN, INPUT);

    analogWrite(ENA, 150);  // Speed (0-255)
    analogWrite(ENB, 150);
    stopCar();
}

void loop() {
    long frontDistance = getDistance(FRONT_TRIG_PIN, FRONT_ECHO_PIN);
    if (frontDistance < DISTANCE_THRESHOLD) {
        stopCar();  // Stop the car if obstacle is detected in front
    }

    if (BT.available()) {  
        char command = BT.read();
        Serial.println(command); // Debugging

        if (command == 'F' && frontDistance >= DISTANCE_THRESHOLD) { 
            forward(); // Move forward if no obstacle in front
        }
        else if (command == 'B' && frontDistance >= DISTANCE_THRESHOLD) { 
            backward(); // Move backward if no obstacle in front
        }
        else if (command == 'L') { 
            left();
        }
        else if (command == 'R') { 
            right();
        }
        else if (command == 'S') { 
            stopCar();
        }
    }
}

long getDistance(int trigPin, int echoPin) {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    
    long duration = pulseIn(echoPin, HIGH);
    long distance = (duration / 2) / 29.1;  // Distance in cm
    
    return distance;
}
void forward() {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
}

void backward() {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
}

void left() {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
}

void right() {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
}

void stopCar() {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
}
```
Pictures and Videos:


Author

Javeria Nadeem

LinkedIn
www.linkedin.com/in/javeria-nadeem-1a9149361
