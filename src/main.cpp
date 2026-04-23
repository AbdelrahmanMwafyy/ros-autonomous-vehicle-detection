#define ROSLIB_SERIALIZATION_BUFFER_SIZE 1024
#include <Wire.h>
#include <WiFi.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include "MotorControl.h"
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <Arduino.h>
#include "WiFi.h"

// ====== WiFi Config ======
const char* ssid = "YOUR_WIFI_SSID";
const char* password = "YOUR_WIFI_PASSWORD";
IPAddress server(192,168,1,100);  // Replace with your ROS master IP

uint16_t serverPort = 11411;

// ROS Node
ros::NodeHandle nh;

// BNO055 sensor
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);  // ID, Address, &Wire

// Messages
std_msgs::Float32 yaw_msg;
ros::Publisher yaw_pub("imu/yaw", &yaw_msg);
geometry_msgs::Pose2D pose_msg;
ros::Publisher pose_pub("robot_pose", &pose_msg);

// Variables for angle and position calculation
float yawAngle = 0;
float initial_yaw = 0;  // Store initial yaw for offset
bool initial_yaw_set = false;
float x_pos = 0.0;
float y_pos = 0.0;
float theta = 0.0;
unsigned long lastTime = 0;

// Robot Kinematics
float wheel_radius = 0.065;  // meters
float half_wheel_base = 0.2647;  // meters

// Motor instances
Motor motor1(25, 26, 13, 18, 1, 1, 0.01, 0, 0,1, false);
Motor motor2(33, 32, 2, 19, 1, 1, 0.01, 0, 2, 3, false);
Motor motor3(34, 14,5, 12, 1, 1, 0.01, 0, 4,5, true);
Motor motor4(36, 39,4,23, 1, 1, 0.009, 0, 6,7, false);

void setupWiFi() {
    Serial.println("Starting WiFi setup...");
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20) {
        delay(500);
        Serial.print(".");
        attempts++;
    }
    
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\nWiFi connected!");
        Serial.print("SSID: ");
        Serial.println(WiFi.SSID());
        Serial.print("IP: ");
        Serial.println(WiFi.localIP());
    } else {
        Serial.println("\nWiFi connection failed!");
        ESP.restart();  // Restart ESP32 if WiFi fails
    }
}

void setZeroAngle() {
    Serial.println("Setting zero angle...");
    
    // Method 1: Use BNO055's built-in offset
    adafruit_bno055_offsets_t current_offsets;
    
    // Get the current orientation
    sensors_event_t event;
    bno.getEvent(&event);
    
    // Store current offsets
    bno.getSensorOffsets(current_offsets);
    
    // Calculate new offset to make current position zero
    current_offsets.gyro_offset_x = current_offsets.gyro_offset_x - (int16_t)(event.orientation.x * 16);
    
    // Apply the new offsets
    bno.setSensorOffsets(current_offsets);
    
    // Method 2: Software offset (backup method)
    initial_yaw = event.orientation.x;
    initial_yaw_set = true;
    
    Serial.printf("Zero angle set. Initial yaw: %.2f\n", initial_yaw);
    
    // Give sensor time to apply new offsets
    delay(100);
}

void initBNO055() {
    Serial.println("Starting BNO055 initialization...");
    
    // Initialize I2C with lower clock speed
    Wire.begin(21, 22);  // SDA, SCL
    Wire.setClock(50000);  // Reduce to 50kHz for stability
    
    delay(1000);  // Give some time after I2C init
    
    int attempts = 0;
    while (!bno.begin() && attempts < 5) {
        Serial.println("Failed to initialize BNO055! Retrying...");
        delay(1000);
        attempts++;
    }
    
    if (attempts >= 5) {
        Serial.println("Could not find a valid BNO055 sensor, check wiring!");
        return;
    }
    
    Serial.println("BNO055 initialized successfully!");
    delay(1000);
    
    bno.setExtCrystalUse(true);
    
    // Get system status
    uint8_t system_status, self_test_result, system_error;
    bno.getSystemStatus(&system_status, &self_test_result, &system_error);
    
    Serial.println("\nBNO055 Status:");
    Serial.printf("System Status: %d\n", system_status);
    Serial.printf("Self Test: %d\n", self_test_result);
    Serial.printf("System Error: %d\n", system_error);
    
    // Wait for initial calibration
    int calibration_attempts = 0;
    while (calibration_attempts < 10) {  // Limit calibration wait time
        uint8_t system, gyro, accel, mag;
        bno.getCalibration(&system, &gyro, &accel, &mag);
        Serial.printf("CALIBRATION - Sys:%d Gyro:%d Acc:%d Mag:%d\n", 
                     system, gyro, accel, mag);
        
        if (system >= 2) {  // Accept if system calibration is at least 2
            Serial.println("Basic calibration achieved!");
            break;
        }
        delay(500);
        calibration_attempts++;
    }
    
    // After calibration, set zero angle
    setZeroAngle();
}

void cmdVelCallback(const geometry_msgs::Twist& msg) {
    float v = msg.linear.x;
    float omega = msg.angular.z;

    float v_left = (v + half_wheel_base * omega) / wheel_radius;
    float v_right = (v - half_wheel_base * omega) / wheel_radius;

    float rad_per_sec_to_rpm = 60.0 / (2.0 * PI);
    v_left *= rad_per_sec_to_rpm;
    v_right *= rad_per_sec_to_rpm;
    
    motor1.setSetpoint(v_right);  // right front
    motor2.setSetpoint(v_right);  // right rear
    motor3.setSetpoint(v_left);   // left front
    motor4.setSetpoint(v_left);   // left rear
}

void stopMotorsCallback(const std_msgs::Bool& msg) {
    if (msg.data) {
        motor1.forceStop();
        motor2.forceStop();
        motor3.forceStop();
        motor4.forceStop();
    }
}

ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("cmd_vel", cmdVelCallback);
ros::Subscriber<std_msgs::Bool> stop_sub("stop_motors", stopMotorsCallback);

void setup() {
    Serial.begin(115200);
    while(!Serial) { ; }
    Serial.println("\nStarting setup...");
    
    // First setup WiFi
    setupWiFi();
    delay(1000);  // Give WiFi some time to stabilize
    
    // Then initialize BNO055
    initBNO055();
    
    // Initialize ROS
    nh.getHardware()->setConnection(server, serverPort);
    nh.initNode();
    nh.advertise(yaw_pub);
    nh.advertise(pose_pub);
    nh.subscribe(cmd_vel_sub);
    nh.subscribe(stop_sub);
    
    lastTime = micros();
    Serial.println("Setup completed!");
}

void loop() {
    // Get orientation data from BNO055
    sensors_event_t event;
    bno.getEvent(&event);
    
    // Get the yaw angle (heading) and apply offset if needed
    if (initial_yaw_set) {
        // Software offset method (backup)
        yawAngle = event.orientation.x - initial_yaw;
        
        // Normalize angle to -180 to +180
        while (yawAngle > 180) yawAngle -= 360;
        while (yawAngle < -180) yawAngle += 360;
    } else {
        yawAngle = event.orientation.x;
    }
    
    // Convert to radians for ROS
    theta = yawAngle * DEG_TO_RAD;
    
    // Update motors
    double speed1 = motor1.update();
    double speed2 = motor2.update();
    double speed3 = motor3.update();
    double speed4 = motor4.update();

    // Calculate robot's velocities from wheel speeds
    float rpm_left = (speed3 + speed4) / 2.0;
    float rpm_right = (speed1 + speed2) / 2.0;

    // Convert RPM to m/s
    float rpm_to_ms = (2.0 * PI * wheel_radius) / 60.0;
    float v_left = rpm_left * rpm_to_ms;
    float v_right = rpm_right * rpm_to_ms;

    // Calculate robot's linear and angular velocities
    float v = (v_right + v_left) / 2.0;                    // Linear velocity
    float omega = (v_right - v_left) / (2 * half_wheel_base); // Angular velocity

    // Calculate time difference for position integration
    unsigned long currentTime = micros();
    float deltaTime = (currentTime - lastTime) / 1000000.0;  // Convert to seconds
    lastTime = currentTime;

    // Update position
    x_pos += v * cos(theta) * deltaTime;
    y_pos += v * sin(theta) * deltaTime;

    // Update and publish messages
    yaw_msg.data = yawAngle;
    yaw_pub.publish(&yaw_msg);

    pose_msg.x = x_pos;
    pose_msg.y = y_pos;
    pose_msg.theta = theta;
    pose_pub.publish(&pose_msg);

    // Debug output
    Serial.printf("Pose - X: %.3f, Y: %.3f, Yaw: %.1f°\n", x_pos, y_pos, yawAngle);
    Serial.printf("Speeds - Left: %.2f, Right: %.2f m/s\n", v_left, v_right);

    // Get calibration status
    uint8_t system, gyro, accel, mag;
    bno.getCalibration(&system, &gyro, &accel, &mag);
    Serial.printf("Cal - Sys:%d Gyro:%d Acc:%d Mag:%d\n", system, gyro, accel, mag);

    nh.spinOnce();
    delay(10);  // 100 Hz update rate
}