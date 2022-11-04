#include <Arduino.h>
#include <Wire.h>

#include "BluetoothSerial.h"

BluetoothSerial SerialBT;

#define BUZZER 27
#define BUZZER_ON LOW
#define BUZZER_OFF HIGH
#define VBAT 34

#define BRAKE 26

#define DIR1 4
#define PWM1 32
#define PWM1_CH 1

#define DIR2 15
#define PWM2 25
#define PWM2_CH 0

#define DIR3 5
#define PWM3 18
#define PWM3_CH 2

#define TIMER_BIT 8
#define BASE_FREQ 20000

#define MPU6050 0x68       // Device address
#define ACCEL_CONFIG 0x1C  // Accelerometer configuration address
#define GYRO_CONFIG 0x1B   // Gyro configuration address

#define PWR_MGMT_1 0x6B
#define PWR_MGMT_2 0x6C

// Sensor output scaling
#define accSens 0   // 0 = 2g, 1 = 4g, 2 = 8g, 3 = 16g
#define gyroSens 1  // 0 = 250rad/s, 1 = 500rad/s, 2 1000rad/s, 3 = 2000rad/s

float Gyro_amount = 0.1;

bool vertical = false;
int balancing_point = 0;

float K1 = 160;
float K2 = 10.50;
float K3 = 0.03;
int loop_time = 10;

float offsetX = -0.84, offsetY = -9.81;
float offsetX2 = -28.47, offsetY2 = -18.87;
float offsetX3 = 29.60, offsetY3 = -17.74;
float offsetX4 = -0.76, offsetY4 = 34.27;
float alpha = 0.74;

/*
// original
float offsetX = -0.99, offsetY = -3.43;
float offsetX2 = -31.24, offsetY2 = -19.05;
float offsetX3 = 30.40, offsetY3 = -19.21;
float offsetX4 = 0.17, offsetY4 = 35.9;
float alpha = 0.74;
*/

int16_t AcX, AcY, AcZ, GyX, GyY, GyZ, gyroX, gyroY, gyroZ, gyroYfilt, gyroZfilt;

int16_t AcX_offset = -940;
int16_t AcY_offset = -200;
int16_t AcZ_offset = 1800;
int16_t GyZ_offset = 0;
int16_t GyY_offset = 0;
int16_t GyX_offset = 0;
int32_t GyZ_offset_sum = 0;
int32_t GyY_offset_sum = 0;
int32_t GyX_offset_sum = 0;

float robot_angleX, robot_angleY, angleX, angleY;
float Acc_angleX, Acc_angleY;
int32_t motor_speed_X;
int32_t motor_speed_Y;

long currentT, previousT_1, previousT_2 = 0;

void writeTo(byte device, byte address, byte value) {
    Wire.beginTransmission(device);
    Wire.write(address);
    Wire.write(value);
    Wire.endTransmission(true);
}

void beep() {
    if (!vertical) {
        digitalWrite(BUZZER, BUZZER_ON);
        delay(50);
        digitalWrite(BUZZER, BUZZER_OFF);
    }
}

void angle_calc() {
    Wire.beginTransmission(MPU6050);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050, 6, true);
    GyX = Wire.read() << 8 | Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
    GyY = Wire.read() << 8 | Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
    GyZ = Wire.read() << 8 | Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

    Wire.beginTransmission(MPU6050);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050, 6, true);
    AcX = Wire.read() << 8 | Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
    AcY = Wire.read() << 8 | Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
    AcZ = Wire.read() << 8 | Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)

    // add mpu6050 offset values
    AcX += AcX_offset;
    AcY += AcY_offset;
    AcZ += AcZ_offset;
    GyZ -= GyZ_offset;
    GyY -= GyY_offset;
    GyX -= GyX_offset;

    robot_angleX += GyZ * loop_time / 1000 / 65.536;
    Acc_angleX = atan2(AcY, -AcX) * 57.2958;  // angle from acc. values * 57.2958 (deg/rad)
    robot_angleX = robot_angleX * Gyro_amount + Acc_angleX * (1.0 - Gyro_amount);

    robot_angleY += GyY * loop_time / 1000 / 65.536;
    Acc_angleY = -atan2(AcZ, -AcX) * 57.2958;  // angle from acc. values * 57.2958 (deg/rad)
    robot_angleY = robot_angleY * Gyro_amount + Acc_angleY * (1.0 - Gyro_amount);

    //  SerialBT.print("AngleX_: "); SerialBT.print(robot_angleX); SerialBT.print(" AngleY_: "); SerialBT.println(robot_angleY);
    // Serial.print("AngleX_: "); Serial.print(robot_angleX); Serial.print(" AngleY_: "); Serial.println(robot_angleY);
    angleX = robot_angleX - offsetX;
    angleY = robot_angleY - offsetY;
    //   SerialBT.print("AngleX: "); SerialBT.print(angleX); SerialBT.print(" AngleY: "); SerialBT.println(angleY);
    //   Serial.print("AngleX: "); Serial.print(angleX); Serial.print(" AngleY: "); Serial.println(angleY);

    if (abs(angleX - offsetX2) < 2 && abs(angleY - offsetY2) < 0.6) {
        balancing_point = 2;
        beep();
        vertical = true;
    } else if (abs(angleX - offsetX3) < 2 && abs(angleY - offsetY3) < 0.6) {
        balancing_point = 3;
        beep();
        vertical = true;
    } else if (abs(angleX - offsetX4) < 0.6 && abs(angleY - offsetY4) < 2) {
        balancing_point = 4;
        beep();
        vertical = true;
    } else if (abs(angleX) < 0.4 && abs(angleY) < 0.4) {
        balancing_point = 1;
        beep();
        vertical = true;
    }
}

void angle_setup() {
    Wire.begin();
    delay(100);
    writeTo(MPU6050, PWR_MGMT_1, 0);
    writeTo(MPU6050, ACCEL_CONFIG, accSens << 3);  // Specifying output scaling of accelerometer
    writeTo(MPU6050, GYRO_CONFIG, gyroSens << 3);  // Specifying output scaling of gyroscope
    delay(100);

    for (int i = 0; i < 1024; i++) {
        angle_calc();
        GyZ_offset_sum += GyZ;
        delay(3);
    }
    GyZ_offset = GyZ_offset_sum >> 10;
    Serial.print("GyZ offset value = ");
    Serial.println(GyZ_offset);

    digitalWrite(BUZZER, BUZZER_ON);
    delay(50);
    digitalWrite(BUZZER, BUZZER_OFF);

    for (int i = 0; i < 1024; i++) {
        angle_calc();
        GyY_offset_sum += GyY;
        delay(3);
    }
    GyY_offset = GyY_offset_sum >> 10;
    Serial.print("GyY offset value = ");
    Serial.println(GyY_offset);

    digitalWrite(BUZZER, BUZZER_ON);
    delay(50);
    digitalWrite(BUZZER, BUZZER_OFF);

    for (int i = 0; i < 1024; i++) {
        angle_calc();
        GyX_offset_sum += GyX;
        delay(3);
    }
    GyX_offset = GyX_offset_sum >> 10;
    Serial.print("GyX offset value = ");
    Serial.println(GyX_offset);

    digitalWrite(BUZZER, BUZZER_ON);
    delay(50);
    digitalWrite(BUZZER, BUZZER_OFF);
    delay(80);
    digitalWrite(BUZZER, BUZZER_ON);
    delay(50);
    digitalWrite(BUZZER, BUZZER_OFF);
}

void pwmSet(uint8_t channel, uint32_t value) {
    ledcWrite(channel, value);
}

void Motor1_control(int sp) {
    if (sp < 0) {
        digitalWrite(DIR1, LOW);
        sp = -sp;
    } else {
        digitalWrite(DIR1, HIGH);
    }
    pwmSet(PWM1_CH, sp > 255 ? 255 : 255 - sp);
}

void Motor2_control(int sp) {
    if (sp < 0) {
        digitalWrite(DIR2, LOW);
        sp = -sp;
    } else {
        digitalWrite(DIR2, HIGH);
    }
    pwmSet(PWM2_CH, sp > 255 ? 255 : 255 - sp);
}

void Motor3_control(int sp) {
    if (sp < 0) {
        digitalWrite(DIR3, LOW);
        sp = -sp;
    } else {
        digitalWrite(DIR3, HIGH);
    }
    pwmSet(PWM3_CH, sp > 255 ? 255 : 255 - sp);
}

void XY_to_threeWay(float pwm_X, float pwm_Y) {
    int16_t m1 = round(0.5 * pwm_X - 0.75 * pwm_Y);
    int16_t m2 = round(0.5 * pwm_X + 0.75 * pwm_Y);
    int16_t m3 = -pwm_X;
    m1 = constrain(m1, -255, 255);
    m2 = constrain(m2, -255, 255);
    m3 = constrain(m3, -255, 255);
    Motor1_control(m1);
    Motor2_control(m2);
    Motor3_control(m3);
}

void battVoltage(double voltage) {
    // Serial.print("batt: "); Serial.println(voltage); //debug
    if (voltage > 8 && voltage <= 9.5) {
        digitalWrite(BUZZER, BUZZER_ON);
    } else {
        digitalWrite(BUZZER, BUZZER_OFF);
    }
}

void printValues() {
    Serial.print("K1: ");
    Serial.print(K1);
    Serial.print(" K2: ");
    Serial.print(K2);
    Serial.print(" K3: ");
    Serial.println(K3, 4);
}

int Tuning() {
    if (!SerialBT.available()) return 0;
    // delay(1);
    char param = SerialBT.read();  // get parameter byte
    if (!SerialBT.available()) return 0;
    char cmd = SerialBT.read();  // get command byte
    // SerialBT.flush();
    switch (param) {
        case 'p':
            if (cmd == '+') K1 += 1;
            if (cmd == '-') K1 -= 1;
            printValues();
            break;
        case 'i':
            if (cmd == '+') K2 += 0.05;
            if (cmd == '-') K2 -= 0.05;
            printValues();
            break;
        case 's':
            if (cmd == '+') K3 += 0.005;
            if (cmd == '-') K3 -= 0.005;
            printValues();
            break;
    }
}

void setup() {
    Serial.begin(115200);
    SerialBT.begin("SEDS-cube");  // Bluetooth device name

    pinMode(BUZZER, OUTPUT);
    digitalWrite(BUZZER, BUZZER_OFF);
    pinMode(BRAKE, OUTPUT);
    digitalWrite(BRAKE, HIGH);

    pinMode(DIR1, OUTPUT);
    ledcSetup(PWM1_CH, BASE_FREQ, TIMER_BIT);
    ledcAttachPin(PWM1, PWM1_CH);
    Motor1_control(0);

    pinMode(DIR2, OUTPUT);
    ledcSetup(PWM2_CH, BASE_FREQ, TIMER_BIT);
    ledcAttachPin(PWM2, PWM2_CH);
    Motor2_control(0);

    pinMode(DIR3, OUTPUT);
    ledcSetup(PWM3_CH, BASE_FREQ, TIMER_BIT);
    ledcAttachPin(PWM3, PWM3_CH);
    Motor3_control(0);

    delay(2000);
    digitalWrite(BUZZER, BUZZER_ON);
    delay(50);
    digitalWrite(BUZZER, BUZZER_OFF);

    SerialBT.println("hallo BT");
    Serial.println("hallo");
    angle_setup();
}

void loop() {
    currentT = millis();

    if (currentT - previousT_1 >= loop_time) {
        Tuning();  // derinimui
        angle_calc();
        if (balancing_point == 1) {
            if (abs(angleX) > 8 || abs(angleY) > 8) vertical = false;
        } else if (balancing_point == 2) {
            angleX -= offsetX2;
            angleY -= offsetY2;
            if (abs(angleY) > 5) vertical = false;
        } else if (balancing_point == 3) {
            angleX -= offsetX3;
            angleY -= offsetY3;
            if (abs(angleY) > 5) vertical = false;
        } else if (balancing_point == 4) {
            angleX -= offsetX4;
            angleY -= offsetY4;
            if (abs(angleX) > 5) vertical = false;
        }

        if (abs(angleX) < 8 || abs(angleY) < 8) {  // fast restore angle
            Gyro_amount = 0.996;
        } else
            Gyro_amount = 0.1;

        if (vertical) {
            digitalWrite(BRAKE, HIGH);
            gyroZ = GyZ / 131.0;  // Convert to deg/s
            gyroY = GyY / 131.0;  // Convert to deg/s
            gyroX = GyX / 131.0;  // Convert to deg/s

            gyroYfilt = alpha * gyroY + (1 - alpha) * gyroYfilt;
            gyroZfilt = alpha * gyroZ + (1 - alpha) * gyroZfilt;

            int pwm_X = constrain(K1 * angleX + K2 * gyroZfilt + K3 * motor_speed_X, -255, 255);
            int pwm_Y = constrain(K1 * angleY + K2 * gyroYfilt + K3 * motor_speed_Y, -255, 255);
            motor_speed_X += pwm_X;
            motor_speed_Y += pwm_Y;

            if (balancing_point == 1) {
                XY_to_threeWay(-pwm_X, -pwm_Y);
            } else if (balancing_point == 2) {
                Motor1_control(pwm_Y);
            } else if (balancing_point == 3) {
                Motor2_control(-pwm_Y);
            } else if (balancing_point == 4) {
                Motor3_control(pwm_X);
            }
        } else {
            XY_to_threeWay(0, 0);
            digitalWrite(BRAKE, LOW);
            motor_speed_X = 0;
            motor_speed_Y = 0;
        }
        previousT_1 = currentT;
    }

    if (currentT - previousT_2 >= 500) {
        battVoltage((double)analogRead(VBAT) / 111);  // 300 kubo plokste
        previousT_2 = currentT;
    }
}
