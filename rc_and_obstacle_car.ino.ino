#include <SoftwareSerial.h>

// Bluetooth Module Pins
SoftwareSerial BT(7, 8);  // HC-05 TX to 7, RX to 8

// Motor Driver Pins
#define ENA 9
#define ENB 6
#define IN1 2
#define IN2 3
#define IN3 4
#define IN4 5

// Ultrasonic Sensor Pins
#define FRONT_TRIG_PIN 10
#define FRONT_ECHO_PIN 11

// Distance threshold (in cm) to stop the car
#define DISTANCE_THRESHOLD 20

void setup() {
    BT.begin(9600);  // Start Bluetooth communication
    Serial.begin(9600); // Debugging
    
    // Motor pins as output
    pinMode(ENA, OUTPUT);
    pinMode(ENB, OUTPUT);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);

    // Ultrasonic sensor pins
    pinMode(FRONT_TRIG_PIN, OUTPUT);
    pinMode(FRONT_ECHO_PIN, INPUT);

    // Set initial motor speed
    analogWrite(ENA, 150);  // Speed (0-255)
    analogWrite(ENB, 150);
    stopCar();
}

void loop() {
    // Check for obstacles in front
    long frontDistance = getDistance(FRONT_TRIG_PIN, FRONT_ECHO_PIN);
    
    // If there's an obstacle in front, stop moving forward or backward
    if (frontDistance < DISTANCE_THRESHOLD) {
        stopCar();  // Stop the car if obstacle is detected in front
    }

    // Handle Bluetooth commands
    if (BT.available()) {  // Check if data is received
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
            stopCar(); // Stop the car when 'S' is pressed
        }
    }
}

// Function to get distance from an ultrasonic sensor
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

// Movement Functions
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
