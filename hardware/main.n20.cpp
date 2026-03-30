//
// Created by timurzakirov on 30.03.2026.
//
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include "SimpleFOC.h"
#include "SimpleFOCDrivers.h"
#include "SimpleDCMotor.h"
#include "encoders/esp32hwencoder/ESP32HWEncoder.h"

// WiFi настройки
const char* ssid = "robotics-lab";
const char* password = "2i9den57sh";

// UDP настройки
WiFiUDP udp;
const int UDP_PORT = 8888;
char packetBuffer[255];
char replyBuffer[255];

#define MON_ENABLE

// =========================
// ОБЩИЕ НАСТРОЙКИ ДЛЯ ОБОИХ МОТОРОВ
// =========================
const int ENCODER_CPR = 1520;
constexpr float VOLTAGE = 1.0f;
constexpr float VEL_LIM = 10;
const int PWM_FREQ = 50000;

constexpr float VEL_PID_P = 2.0f;
constexpr float VEL_PID_I = 20.0f;
constexpr float VEL_PID_D = 0.001f;
constexpr float VEL_PID_LIM = 400.0f;
constexpr float VEL_LPF_TF = 0.01f;

constexpr float EPSILON = 0.001f;

// Плавность изменения целевой скорости
constexpr float TARGET_ACCEL = 10.0f;   // рад/с^2

// =========================
// СЮДА ВПИСАТЬ СВОИ ПИНЫ
// =========================
constexpr int M1_PWM_PIN   = 0;   // PWM мотора 1
constexpr int M1_DIR1_PIN  = 1;   // DIR1 мотора 1
constexpr int M1_DIR2_PIN  = 2;   // DIR2 мотора 1
constexpr int M1_ENC_A_PIN = 3;   // Encoder A мотора 1
constexpr int M1_ENC_B_PIN = 4;   // Encoder B мотора 1

constexpr int M2_PWM_PIN   = 15;   // PWM мотора 2
constexpr int M2_DIR1_PIN  = 18;   // DIR1 мотора 2
constexpr int M2_DIR2_PIN  = 19;   // DIR2 мотора 2
constexpr int M2_ENC_A_PIN = 5;   // Encoder A мотора 2
constexpr int M2_ENC_B_PIN = 14;   // Encoder B мотора 2

// Глобальные переменные для статуса
float current_velocity_left = 0;
float current_voltage_q_left = 0;
float current_voltage_d_left = 0;
float current_angle_left = 0;
float current_target_left = 0;

float current_velocity_right = 0;
float current_voltage_q_right = 0;
float current_voltage_d_right = 0;
float current_angle_right = 0;
float current_target_right = 0;

bool motors_enabled = false;

// Цели от UDP
float requested_target_left = 0;
float requested_target_right = 0;

// Плавные цели
float ramp_target_left = 0;
float ramp_target_right = 0;

// Глобальные переменные для UDP
IPAddress remoteIp;
unsigned int remotePort;

class CustomMotor : public DCMotor {
public:
    using DCMotor::DCMotor;

    virtual void setPhaseVoltage(float Uq, float Ud, float angle_el) override {
        if (enabled) {
            if (fabs(target) < EPSILON) {
                driver->setPwm(0);
                driver->disable();
            } else if (fabs(Uq) < EPSILON) {
                driver->setPwm(0);
            } else if ((target > 0 && Uq < 0) || (target < 0 && Uq > 0)) {
                driver->setPwm(0);
                driver->disable();
            } else {
                driver->setPwm(Uq);
            }
        }
        _UNUSED(Ud);
        _UNUSED(angle_el);
    }
};

class CustomDriver : public DCDriver1PWM2Dir {
public:
    using DCDriver1PWM2Dir::DCDriver1PWM2Dir;

    virtual void setPwm(float U) override {
        DCDriver1PWM2Dir::setPwm(U);
    }
};

Commander commander = Commander(Serial);

// Левый мотор
CustomMotor motor_left = CustomMotor();
CustomDriver driver_left = CustomDriver(M1_PWM_PIN, M1_DIR1_PIN, M1_DIR2_PIN);
ESP32HWEncoder sensor_left = ESP32HWEncoder(M1_ENC_A_PIN, M1_ENC_B_PIN, ENCODER_CPR);

// Правый мотор
CustomMotor motor_right = CustomMotor();
CustomDriver driver_right = CustomDriver(M2_PWM_PIN, M2_DIR1_PIN, M2_DIR2_PIN);
ESP32HWEncoder sensor_right = ESP32HWEncoder(M2_ENC_A_PIN, M2_ENC_B_PIN, ENCODER_CPR);

void onMotorLeft(char* cmd){ commander.motor(&motor_left, cmd); }
void onMotorRight(char* cmd){ commander.motor(&motor_right, cmd); }

// Функция отправки UDP ответа
void sendUDPResponse(String message) {
    udp.beginPacket(remoteIp, remotePort);
    udp.print(message);
    udp.endPacket();
}

// Формат:
// SET_SPEED:left,right
// Пример:
// SET_SPEED:5.0,5.0
// SET_SPEED:-3.0,3.0
void processUDPCommand(String command) {
    command.trim();
    Serial.println("UDP команда: " + command);

    if (command.startsWith("SET_SPEED:")) {
        String data = command.substring(10);
        int commaPos = data.indexOf(',');

        if (commaPos > 0) {
            float leftSpeed = data.substring(0, commaPos).toFloat();
            float rightSpeed = data.substring(commaPos + 1).toFloat();

            requested_target_left = leftSpeed;
            requested_target_right = rightSpeed;

            sendUDPResponse("OK: Speed set to " + String(leftSpeed) + "," + String(rightSpeed));
        } else {
            sendUDPResponse("ERROR: Use SET_SPEED:left,right");
        }

    } else if (command == "GET_STATUS") {
        String status =
            String(current_velocity_left) + "," +
            String(current_target_left) + "," +
            String(current_voltage_q_left) + "," +
            String(current_voltage_d_left) + "," +
            String(current_angle_left) + "," +
            String(current_velocity_right) + "," +
            String(current_target_right) + "," +
            String(current_voltage_q_right) + "," +
            String(current_voltage_d_right) + "," +
            String(current_angle_right) + "," +
            String((motor_left.enabled && motor_right.enabled) ? 1 : 0);

        sendUDPResponse(status);

    } else if (command == "ENABLE") {
        motor_left.enable();
        motor_right.enable();
        sendUDPResponse("OK: Motors enabled");

    } else if (command == "DISABLE") {
        motor_left.disable();
        motor_right.disable();
        sendUDPResponse("OK: Motors disabled");

    } else if (command == "STOP") {
        requested_target_left = 0;
        requested_target_right = 0;
        sendUDPResponse("OK: Motors stopped");

    } else if (command.startsWith("CMDL:")) {
        String cmd = command.substring(5);
        char* cmdStr = (char*)cmd.c_str();
        onMotorLeft(cmdStr);
        sendUDPResponse("OK: Left command executed");

    } else if (command.startsWith("CMDR:")) {
        String cmd = command.substring(5);
        char* cmdStr = (char*)cmd.c_str();
        onMotorRight(cmdStr);
        sendUDPResponse("OK: Right command executed");

    } else if (command == "PING") {
        sendUDPResponse("PONG");

    } else {
        sendUDPResponse("ERROR: Unknown command");
    }
}

void initWiFi() {
    Serial.print("Подключение к WiFi");
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);

    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20) {
        delay(500);
        Serial.print(".");
        attempts++;
    }

    if (WiFi.status() == WL_CONNECTED) {
        Serial.println();
        Serial.println("WiFi подключен!");
        Serial.print("IP адрес: ");
        Serial.println(WiFi.localIP());
    } else {
        Serial.println();
        Serial.println("WiFi не подключен! Запускаем AP режим...");
        WiFi.softAP("ESP32_Motor_Control", "12345678");
        Serial.print("AP IP адрес: ");
        Serial.println(WiFi.softAPIP());
    }
}

void initUDP() {
    udp.begin(UDP_PORT);
    Serial.print("UDP сервер запущен на порту: ");
    Serial.println(UDP_PORT);
}

void initMotorStack(char c, CustomMotor &motor, CustomDriver &driver, ESP32HWEncoder &sensor) {
    driver.voltage_power_supply = VOLTAGE;
    driver.voltage_limit = VOLTAGE;
    driver.pwm_frequency = PWM_FREQ;
    driver.init();

    sensor.init();
    motor.linkDriver(&driver);
    motor.linkSensor(&sensor);

    motor.voltage_limit = VOLTAGE;
    motor.velocity_limit = VEL_LIM;
    motor.controller = MotionControlType::velocity;
    motor.torque_controller = TorqueControlType::voltage;
    motor.init();

    motor.PID_velocity.P = VEL_PID_P;
    motor.PID_velocity.I = VEL_PID_I;
    motor.PID_velocity.D = VEL_PID_D;
    motor.PID_velocity.output_ramp = VEL_PID_LIM;
    motor.LPF_velocity.Tf = VEL_LPF_TF;

    #ifdef MON_ENABLE
    motor.useMonitoring(Serial);
    motor.monitor_downsample = 1;
    motor.monitor_variables = _MON_TARGET | _MON_VOLT_Q | _MON_VEL | _MON_ANGLE;
    motor.monitor_start_char = c;
    #endif

    motor.target = 0.0f;
    motor.enable();
}

void setup() {
    Serial.begin(115200);
    while (!Serial) { delay(10); };
    SimpleFOCDebug::enable(&Serial);
    Serial.println("ESP32 2 Motor Control System with UDP");

    // Инициализация обоих моторов
    initMotorStack('L', motor_left, driver_left, sensor_left);
    initMotorStack('R', motor_right, driver_right, sensor_right);

    commander.add('L', onMotorLeft, "left motor");
    commander.add('R', onMotorRight, "right motor");

    // Инициализация WiFi и UDP
    initWiFi();
    initUDP();

    Serial.println("Система готова!");
    Serial.println("UDP команды:");
    Serial.println("  SET_SPEED:left,right - установить скорости");
    Serial.println("  GET_STATUS           - получить статус");
    Serial.println("  ENABLE/DISABLE       - включить/выключить");
    Serial.println("  STOP                 - остановить");
    Serial.println("  CMDL:<cmd>           - команда левому мотору");
    Serial.println("  CMDR:<cmd>           - команда правому мотору");
    Serial.println("  PING                 - проверка связи");
}

uint32_t last_ts = 0;
uint32_t last_ramp_us = 0;

void loop() {
    // Плавное изменение целей
    uint32_t now_us = micros();
    float dt = (now_us - last_ramp_us) * 1e-6f;
    last_ramp_us = now_us;

    if (dt < 0 || dt > 0.1f) dt = 0.001f;

    float max_step = TARGET_ACCEL * dt;

    float diff_left = requested_target_left - ramp_target_left;
    if (diff_left > max_step) {
        ramp_target_left += max_step;
    } else if (diff_left < -max_step) {
        ramp_target_left -= max_step;
    } else {
        ramp_target_left = requested_target_left;
    }

    float diff_right = requested_target_right - ramp_target_right;
    if (diff_right > max_step) {
        ramp_target_right += max_step;
    } else if (diff_right < -max_step) {
        ramp_target_right -= max_step;
    } else {
        ramp_target_right = requested_target_right;
    }

    motor_left.target = ramp_target_left;
    motor_right.target = ramp_target_right;

    motor_left.move();
    motor_right.move();

    commander.run();

    // Обновляем глобальные переменные для статуса
    current_velocity_left = motor_left.shaft_velocity;
    current_target_left = motor_left.target;
    current_voltage_q_left = motor_left.voltage.q;
    current_voltage_d_left = motor_left.voltage.d;
    current_angle_left = motor_left.shaft_angle;

    current_velocity_right = motor_right.shaft_velocity;
    current_target_right = motor_right.target;
    current_voltage_q_right = motor_right.voltage.q;
    current_voltage_d_right = motor_right.voltage.d;
    current_angle_right = motor_right.shaft_angle;

    motors_enabled = motor_left.enabled && motor_right.enabled;

    // Мониторинг в Serial
    uint32_t now = micros();
    if ((now - last_ts >= 10000) || (now < last_ts)) {
        motor_left.monitor();
        motor_right.monitor();
        last_ts = now;
    }

    // Обработка UDP пакетов
    int packetSize = udp.parsePacket();
    if (packetSize) {
        remoteIp = udp.remoteIP();
        remotePort = udp.remotePort();

        int len = udp.read(packetBuffer, 255);
        if (len > 0) {
            packetBuffer[len] = 0;
            String command = String(packetBuffer);
            processUDPCommand(command);
        }
    }
}