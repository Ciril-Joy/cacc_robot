#define RPWM 3  // Right PWM (Drive Motor)
#define LPWM 4  // Left PWM (Drive Motor)
#define REN 7   // Right Enable (Drive Motor)
#define LEN 8   // Left Enable (Drive Motor)

#define sLPWM 9  // Steering Left PWM signal
#define sRPWM 10 // Steering Right PWM signal
#define sLEN 11  // Steering Left Enable signal
#define sREN 12  // Steering Right Enable signal

void setup() {
    pinMode(RPWM, OUTPUT);
    pinMode(LPWM, OUTPUT);
    pinMode(REN, OUTPUT);
    pinMode(LEN, OUTPUT);

    pinMode(sLPWM, OUTPUT);
    pinMode(sRPWM, OUTPUT);
    pinMode(sLEN, OUTPUT);
    pinMode(sREN, OUTPUT);

    digitalWrite(sLEN, HIGH); // Enable steering motor
    digitalWrite(sREN, HIGH);

    Serial.begin(9600);
}

void loop() {
    if (Serial.available() > 0) {
        char command = Serial.read();
        controlMotors(command);
    }
}

void controlMotors(char command) {
    int speed = 100;  // Adjust speed (0-255)
    int steer_speed = 200; // Steering motor speed

    switch (command) {
        case 'w': // Move Forward
            digitalWrite(REN, LOW); // Enable drive motorsc
            digitalWrite(LEN, HIGH);
            analogWrite(RPWM, speed);
            analogWrite(LPWM, 0);
            break;
        case 's': // Move Backward
            digitalWrite(REN, HIGH); // Enable drive motorsc
            digitalWrite(LEN, HIGH);
            analogWrite(RPWM, 0);
            analogWrite(LPWM, speed);
            break;
        case 'a': // Turn Left (Brief Steering Adjustment)
            //digitalWrite(sLPWM, HIGH);
            //digitalWrite(sRPWM, LOW);
            analogWrite(sLPWM, steer_speed);
            analogWrite(sRPWM, 0);
            delay(1000); // Adjust based on testing
            analogWrite(sLPWM, 0);
            analogWrite(sLPWM, 0);
            break;
        case 'd': // Turn Right (Brief Steering Adjustment)
            //digitalWrite(sLPWM, LOW);
            //digitalWrite(sRPWM, HIGH);
            analogWrite(sLPWM, 0);
            analogWrite(sRPWM, steer_speed);
            delay(1000); // Adjust based on testing
            analogWrite(sLPWM, 0);
            analogWrite(sRPWM, 0);
            break;
        case 'x': // Stop All
            analogWrite(RPWM, 0);
            analogWrite(LPWM, 0);
            analogWrite(sLPWM, 0);
            analogWrite(sRPWM, 0);
            break;
    }
}
