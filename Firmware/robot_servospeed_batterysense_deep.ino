#include <WiFi.h>
#include <WiFiUdp.h>
#include <ESP32Servo.h>

// ======== CONFIGURATION ======== //
const char* ssid = "VATSAL0958";
const char* ssid_password = "Vatsal0958";
const int UDP_PORT = 8888;

const int SERVO_L_PIN = 4;  // Left servo GPIO
const int ELECTROMAGNET = 22;
const int SERVO_R_PIN = 14;  // Right servo GPIO
const int NEUTRAL = 90;      // Stop pulse width (calibrate!)
const uint32_t SERVO_TIMEOUT_MS = 500;  // 1 second timeout
const int BATTERY_PIN = 35;  // Analog pin for battery voltage
const float LOW_VOLTAGE_THRESHOLD = 6.4;  // Voltage threshold in volts
const float CHARGED_VOLTAGE_THRESHOLD = 7.4;  // Fully charged voltage
const uint32_t BATTERY_CHECK_INTERVAL = 5000;  // Check every 5 seconds

Servo servo_L, servo_R;
WiFiUDP udp;

// Servo control variables
volatile uint32_t last_command_time = 0;
volatile bool servos_attached = false;

// Battery monitoring variables
volatile bool low_battery = false;
volatile bool charging_mode = false;
TaskHandle_t batteryTaskHandle = NULL;

void setup() {
  Serial.begin(115200);
  
  // Initialize servos in detached state
  stopMotors();
  detachServos();
  pinMode(ELECTROMAGNET, OUTPUT);
  
  // WiFi connection
  WiFi.begin(ssid, ssid_password);
  Serial.print("Connecting to WiFi");
  while(WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  
  // Network info
  Serial.println("\nConnected!");
  Serial.print("Robot IP: ");
  Serial.println(WiFi.localIP());
  Serial.print("UDP Port: ");
  Serial.println(UDP_PORT);

  // Start UDP server
  udp.begin(UDP_PORT);

  // Create battery monitoring task
  xTaskCreatePinnedToCore(
    batteryMonitorTask,    // Task function
    "BatteryMonitor",      // Name of task
    2000,                 // Stack size (bytes)
    NULL,                 // Parameter to pass
    1,                    // Task priority
    &batteryTaskHandle,   // Task handle
    0                     // Core to run on (0 or 1)
  );
}

void loop() {
  checkServoTimeout();  // Handle timeout detection
  
  int packetSize = udp.parsePacket();
  if(packetSize) {
    String command = udp.readString();
    command.trim();    
    Serial.print("CMD: ");
    if(command == "M"){
      digitalWrite(ELECTROMAGNET, HIGH);
    }else if(command == "D"){
      digitalWrite(ELECTROMAGNET, LOW);
    }
    Serial.println(command);

    int comma = command.indexOf(',');
    if(comma != -1) {
      int L = command.substring(0, comma).toInt();
      int R = command.substring(comma+1).toInt();
      setMotorSpeeds(L, R);
    }
  }
}

void batteryMonitorTask(void *pvParameters) {
  (void) pvParameters;
  
  // Voltage divider calibration - adjust these based on your voltage divider circuit
  const float R1 = 560000.0;  // Resistor 1 value (ohms)
  const float R2 = 330000.0;  // Resistor 2 value (ohms)
  const float voltageDividerRatio = R2 / (R1 + R2);
  const float adcReferenceVoltage = 3.3;  // ESP32 ADC reference voltage
  const int adcMaxValue = 4095;          // ESP32 12-bit ADC max value
  
  while(1) {
    // Read battery voltage
    int rawValue = analogRead(BATTERY_PIN);
    float voltageAtPin = (rawValue / (float)adcMaxValue) * adcReferenceVoltage;
    float batteryVoltage = voltageAtPin / voltageDividerRatio;
    
    Serial.print("Battery Voltage: ");
    Serial.print(batteryVoltage);
    Serial.println("V");
    
    // Low battery detection
    if(batteryVoltage < LOW_VOLTAGE_THRESHOLD && !low_battery) {
      low_battery = true;
      charging_mode = true;  // Enter charging mode
      Serial.println("Low battery detected! Sending alert...");
      
      // Send alert to computer
      if(WiFi.status() == WL_CONNECTED) {
        udp.beginPacket(udp.remoteIP(), udp.remotePort());
        udp.print("LOW_BATTERY");
        udp.endPacket();
      }
      
      // Stop motors to conserve power
      stopMotors();
      detachServos();
    } 
    // Charging complete detection
    else if(charging_mode && batteryVoltage >= CHARGED_VOLTAGE_THRESHOLD) {
      low_battery = false;
      charging_mode = false;
      Serial.println("Battery fully charged! Sending notification...");
      
      // Send charged notification to computer
      if(WiFi.status() == WL_CONNECTED) {
        udp.beginPacket(udp.remoteIP(), udp.remotePort());
        udp.print("C");  // "C" for Charged
        udp.endPacket();
      }
    }
    // Hysteresis for low battery clear (optional)
    else if(!charging_mode && batteryVoltage >= LOW_VOLTAGE_THRESHOLD + 0.2) {
      low_battery = false;
    }
    
    vTaskDelay(pdMS_TO_TICKS(BATTERY_CHECK_INTERVAL));
  }
}

void setMotorSpeeds(int speed_L, int speed_R) {
  // Don't allow motor movement if battery is low
  if(low_battery) {
    Serial.println("Battery low - motors disabled");
    return;
  }
  
  // Attach servos if not already attached
  if(!servos_attached) {
    servo_L.attach(SERVO_L_PIN);
    servo_R.attach(SERVO_R_PIN);
    servos_attached = true;
  }

  // Update servo positions
  servo_L.write(speed_L);
  servo_R.write(speed_R);
  
  // Update last command time
  last_command_time = millis();
}

void checkServoTimeout() {
  if(servos_attached && (millis() - last_command_time > SERVO_TIMEOUT_MS)) {
    stopMotors();
    detachServos();
    Serial.println("Servo timeout - motors detached");
  }
}

void stopMotors() {
  if(servos_attached) {
    servo_L.write(NEUTRAL);
    servo_R.write(NEUTRAL);
  }
}

void detachServos() {
  if(servos_attached) {
    servo_L.write(90);
    servo_R.write(90);
    servos_attached = false;
  }
}