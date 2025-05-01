#define RPWM 5  // Right PWM (Drive Motor)
#define LPWM 6  // Left PWM (Drive Motor)
#define REN 7   // Right Enable (Drive Motor)
#define LEN 8   // Left Enable (Drive Motor)

#define sLPWM // Steering Left PWM signal
#define sRPWM // Steering Right PWM signal
#define sLEN // Steering Left Enable signal
#define sREN // Steering Right Enable signal

void setup() {
    pinMode(RPWM, OUTPUT);
    pinMode(LPWM, OUTPUT);
    pinMode(REN, OUTPUT);
    pinMode(LEN, OUTPUT);
    
    pinMode(REN, OUTPUT);
    pinMode(LEN, OUTPUT);
    pinMode(REN, OUTPUT);
    pinMode(LEN, OUTPUT);

    digitalWrite(REN, HIGH); // Enable drive motor
    digitalWrite(LEN, HIGH);
    
    Serial.begin(9600);
}

void loop() {
    if (Serial.available() > 0) {
        char command = Serial.read();
        controlMotors(command);
    }
}

void controlMotors(char command) {
    int speed = 150;  // Adjust speed (0-255)
    int steer_speed = 100; // Steering motor speed

    switch (command) {
        case 'w': // Move Forward
            analogWrite(RPWM, speed);
            //analogWrite(LPWM, 0);
            break;
        case 's': // Move Backward
            analogWrite(RPWM, 0);
            analogWrite(LPWM, speed);
            break;
        case 'a': // Turn Left (Brief Steering Adjustment)
            digitalWrite(STEER1, HIGH);
            digitalWrite(STEER2, LOW);
            analogWrite(STEER_PWM, steer_speed);
            delay(500); // Adjust based on testing
            analogWrite(STEER_PWM, 0);
            break;
        case 'd': // Turn Right (Brief Steering Adjustment)
            digitalWrite(STEER1, LOW);
            digitalWrite(STEER2, HIGH);
            analogWrite(STEER_PWM, steer_speed);
            delay(500); // Adjust based on testing
            analogWrite(STEER_PWM, 0);
            break;
        case 'x': // Stop All
            analogWrite(RPWM, 0);
            analogWrite(LPWM, 0);
            analogWrite(STEER_PWM, 0);
            break;
    }
}