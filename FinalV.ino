#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"
#include "BluetoothSerial.h"
#include <MPU6050.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

MAX30105 particleSensor;
MPU6050 mpu;
BluetoothSerial SerialBT;

#define MAX_SAMPLES 100
uint32_t irBuffer[MAX_SAMPLES];
uint32_t redBuffer[MAX_SAMPLES];

int32_t spo2;
int8_t validSpO2;
int32_t heartRate;
int8_t validHeartRate;

#define HR_AVG_WINDOW 25  // Heart rate average window in seconds (25 seconds)
#define SPO2_AVG_WINDOW 30  // SpO2 average window in seconds (30 seconds)

// Timing for HR and SpO2 calculations
unsigned long lastHRUpdate = 0;
unsigned long lastSpO2Update = 0;

int hrSamples = 0, spo2Samples = 0;
long hrSum = 0, spo2Sum = 0;

int averagedHR = -1;
int averagedSpO2 = -1;

bool isStable = true;
void setup() {
  Serial.begin(115200);
  SerialBT.begin("ESP32_PulseOximeter");

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("OLED not found!");
    while (1);
  }

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(10, 10);
  display.println("Initializing...");
  display.display();
  delay(2000);

  if (!particleSensor.begin(Wire, I2C_SPEED_STANDARD)) {
    Serial.println("MAX30102 NOT detected!");
    while (1);
  }

  particleSensor.setup(0xFF, 4, 2, 400, 411, 16384);
  Serial.println("MAX30102 Ready!");

  Wire.begin();
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 NOT detected!");
    while (1);
  }
  Serial.println("MPU6050 Ready!");
}

bool isDeviceStable() {
  int16_t ax, ay, az;
  mpu.getAcceleration(&ax, &ay, &az);
  float magnitude = sqrt(ax * ax + ay * ay + az * az);
  return (magnitude > 15000 && magnitude < 18000);
}

void showOLED(int spo2, int hr) {
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(10, 5);
  display.println("Health Monitor");

  display.setTextSize(1);
  display.setCursor(10, 30);
  display.print("SpO2: ");
  display.print(spo2);
  display.println(" %");

  display.setTextSize(1);
  display.setCursor(10, 45);
  display.print("Heart Rate: ");
  display.print(hr);
  display.println(" BPM");

  display.display();
}

void loop() {
  isStable = isDeviceStable();

  if (!isStable) {
    Serial.println("Device moving... Skipping.");
    delay(1000);
    return;
  }

  for (int i = 0; i < MAX_SAMPLES; i++) {
    while (!particleSensor.check()) delay(1);
    redBuffer[i] = particleSensor.getRed();
    irBuffer[i] = particleSensor.getIR();
  }

  maxim_heart_rate_and_oxygen_saturation(irBuffer, MAX_SAMPLES, redBuffer, &spo2, &validSpO2, &heartRate, &validHeartRate);

  unsigned long currentMillis = millis();

  // Calculate the heart rate average every 25 seconds
  if (heartRate >= 40 && heartRate <= 140) {
    hrSamples++;
    hrSum += heartRate;
  }

  // Calculate the SpO2 average every 30 seconds
  if (spo2 >= 90 && spo2 <= 100) {
    spo2Samples++;
    spo2Sum += spo2;
  }

  // Update Heart Rate every 25 seconds
  if (currentMillis - lastHRUpdate >= HR_AVG_WINDOW * 1000) {
    if (hrSamples > 0) {
      averagedHR = hrSum / hrSamples;  // Calculate average HR
    }
    
    // Reset HR calculation variables after 25 seconds
    hrSamples = 0;
    hrSum = 0;
    lastHRUpdate = currentMillis; // Set last HR update time

    // Print Heart Rate to Serial Monitor and Bluetooth
    Serial.println("HR:" + String(averagedHR));
    SerialBT.println("HR:" + String(averagedHR));
  }

  // Update SpO2 every 30 seconds
  if (currentMillis - lastSpO2Update >= SPO2_AVG_WINDOW * 1000) {
    if (spo2Samples > 0) {
      averagedSpO2 = spo2Sum / spo2Samples;  // Calculate average SpO2
    }

    // Reset SpO2 calculation variables after 30 seconds
    spo2Samples = 0;
    spo2Sum = 0;
    lastSpO2Update = currentMillis; // Set last SpO2 update time

    // Print SpO2 to Serial Monitor and Bluetooth
    Serial.println("SPO2:" + String(averagedSpO2));
    SerialBT.println("SPO2:" + String(averagedSpO2));
  }

  // Show values on OLED
  showOLED(averagedSpO2, averagedHR);

  delay(1000);  // Delay for 1 second before the next loop iteration
}
