#include <Arduino.h>
#include <PID_v1.h>
#include <AceButton.h>
#include <Adafruit_ST7789.h>
#include <Adafruit_GFX.h>
using namespace ace_button;

void updateDisplay();

int MOTOR_PIN_1 = 9;
int MOTOR_PIN_2 = 10;
int ENCODER_PIN = A0;
int pinD1 = 1;

AceButton buttonD1(pinD1);
Adafruit_ST7789 display = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

int ch1 = 0;
int ch2 = 1;
int SLOTS_PER_REVOLUTION = 5;
int sensorMin = 4095;
int sensorMax = 0;
int lightThreshold = 1600;
int buffer = 100;

bool slotDetected = false;

unsigned long previousPulseMicros = 0;
unsigned long lastPulseMillis = 0;
unsigned long pulsesSincePrint = 0;

double speedOptions[] = {20.0, 30.0, 40.0};

int NUM_SPEEDS = 3;
int speedIndex = 0;

double targetRPM = 20.0;
double actualRPM = 0.0;
double pidOutput = 105.0;

double Kp = 0.60;
double Ki = 0.15;
double Kd = 0.02;

PID motorPID(&actualRPM, &pidOutput, &targetRPM, Kp, Ki, Kd, DIRECT);

double motorPower = 105.0;

int MIN_MOTOR_POWER = 50;
int MAX_MOTOR_POWER = 150;
int STARTUP_POWER = 125;
int POWER_CHANGE_STEP = 2;

unsigned long lastDisplayTime = 0;
unsigned long lastPrintTime = 0;

int DISPLAY_INTERVAL = 250;
int PRINT_INTERVAL = 500;

void ChangeSpeed(AceButton *button, uint8_t eventType, uint8_t buttonState) {

  if(eventType == (uint8_t)AceButton::kEventLongPressed) {

    if(button->getPin() == pinD1) {
      speedIndex++;

      if(speedIndex >= NUM_SPEEDS) {
        speedIndex = 0;
      }

      targetRPM = speedOptions[speedIndex];

      Serial.println();
      Serial.print("NEW TARGET = ");
      Serial.print(targetRPM, 0);
      Serial.println(" RPM");
      Serial.println();

      updateDisplay();
    }
  }
}

void startMotor()
{
  ledcWrite(ch1, STARTUP_POWER);
  ledcWrite(ch2, 0);

  delay(400);

  motorPower = 105;
  pidOutput = 105;

  ledcWrite(ch1, (int)motorPower);
  ledcWrite(ch2, 0);
}

void calibrateEncoder()
{
  Serial.println();
  Serial.println("Calibrating encoder...");
  Serial.println("Wheel will spin for 4 seconds.");
  Serial.println();

  sensorMin = 4095;
  sensorMax = 0;

  ledcWrite(ch1, 110);
  ledcWrite(ch2, 0);

  unsigned long startTime = millis();

  while(millis() - startTime < 4000)
  {
    int value = analogRead(ENCODER_PIN);

    if(value < sensorMin)
    {
      sensorMin = value;
    }

    if(value > sensorMax)
    {
      sensorMax = value;
    }

    delay(1);
  }

  int sensorRange = sensorMax - sensorMin;

  lightThreshold = (sensorMin + sensorMax) / 2;

  buffer = sensorRange / 8;
  buffer = constrain(buffer, 20, 250);

  Serial.print("Minimum: ");
  Serial.println(sensorMin);
  Serial.print("Maximum: ");
  Serial.println(sensorMax);
  Serial.print("Threshold: ");
  Serial.println(lightThreshold);
  Serial.print("Buffer: ");
  Serial.println(buffer);
  Serial.println();

  int currentLight = analogRead(ENCODER_PIN);

  slotDetected = currentLight > lightThreshold;

  previousPulseMicros = 0;
  lastPulseMillis = millis();
}

void readEncoder()
{
  int lightValue = analogRead(ENCODER_PIN);

  if(!slotDetected && lightValue > lightThreshold + buffer)
  {
    slotDetected = true;

    pulsesSincePrint++;

    unsigned long now = micros();

    if(previousPulseMicros != 0)
    {
      unsigned long pulseTime = now - previousPulseMicros;

      double newRPM = 60000000.0 / ((double)pulseTime * SLOTS_PER_REVOLUTION);

      if(actualRPM <= 0)
      {
        actualRPM = newRPM;
      }
      else
      {
        actualRPM = actualRPM * 0.70 + newRPM * 0.30;
      }
    }

    previousPulseMicros = now;

    lastPulseMillis = millis();
  }

  if(slotDetected && lightValue < lightThreshold - buffer)
  {
    slotDetected = false;
  }
}

void updateMotor()
{
  if(motorPID.Compute())
  {
    if(pidOutput > motorPower + POWER_CHANGE_STEP)
    {
      motorPower += POWER_CHANGE_STEP;
    }

    else if(pidOutput < motorPower - POWER_CHANGE_STEP)
    {
      motorPower -= POWER_CHANGE_STEP;
    }

    else
    {
      motorPower = pidOutput;
    }

    motorPower = constrain(motorPower, MIN_MOTOR_POWER, MAX_MOTOR_POWER);

    ledcWrite(ch1, (int)motorPower);
    ledcWrite(ch2, 0);
  }
}

void updateDisplay()
{
  display.fillScreen(ST77XX_BLACK);
  display.setCursor(10, 12);
  display.setTextColor(ST77XX_WHITE);
  display.setTextSize(2);
  display.println("Target");
  display.setTextColor(ST77XX_YELLOW);
  display.setTextSize(3);
  display.print(targetRPM, 0);
  display.println(" RPM");
  display.setTextColor(ST77XX_WHITE);
  display.setTextSize(2);
  display.println("");
  display.println("Actual");
  display.setTextColor(ST77XX_GREEN);
  display.setTextSize(3);
  display.print(actualRPM, 1);
  display.println(" RPM");
}

void setup()
{
  Serial.begin(115200);

  delay(1000);

  pinMode(pinD1, INPUT_PULLDOWN);
  buttonD1.init(pinD1, LOW);
  ButtonConfig* buttonConfigD1 = buttonD1.getButtonConfig();
  buttonConfigD1->setEventHandler(ChangeSpeed);
  buttonConfigD1->setFeature(ButtonConfig::kFeatureLongPress);
  buttonConfigD1->setLongPressDelay(1000);

  pinMode(ENCODER_PIN, INPUT);

  analogReadResolution(12);
  analogSetPinAttenuation(ENCODER_PIN, ADC_11db);

  pinMode(TFT_BACKLITE, OUTPUT);
  digitalWrite(TFT_BACKLITE, HIGH);

  pinMode(TFT_I2C_POWER, OUTPUT);
  digitalWrite(TFT_I2C_POWER, HIGH);

  display.init(135, 240);
  display.setRotation(3);
  display.fillScreen(ST77XX_BLACK);

  ledcSetup(ch1, 5000, 8);
  ledcSetup(ch2, 5000, 8);

  ledcAttachPin(MOTOR_PIN_1, ch1);
  ledcAttachPin(MOTOR_PIN_2, ch2);

  ledcWrite(ch1, 0);
  ledcWrite(ch2, 0);

  delay(500);

  calibrateEncoder();

  motorPID.SetOutputLimits(MIN_MOTOR_POWER, MAX_MOTOR_POWER);
  motorPID.SetSampleTime(1000);
  motorPID.SetMode(AUTOMATIC);

  startMotor();

  lastPulseMillis = millis();
  lastDisplayTime = millis();
  lastPrintTime = millis();

  updateDisplay();

  Serial.println();
  Serial.println("============================");
  Serial.println("MOTOR SPEED CONTROL");
  Serial.println("============================");
  Serial.println();
  Serial.println("Hold D1 for 1 second");
  Serial.println("20 -> 30 -> 40 -> 20 RPM");
  Serial.println();
}

void loop()
{
  buttonD1.check();

  readEncoder();

  if(millis() - lastPulseMillis > 3000)
  {
    actualRPM = 0;
  }

  updateMotor();

  if(millis() - lastDisplayTime >= DISPLAY_INTERVAL)
  {
    lastDisplayTime = millis();
    updateDisplay();
  }

  if(millis() - lastPrintTime >= PRINT_INTERVAL)
  {
    lastPrintTime = millis();

    int lightValue = analogRead(ENCODER_PIN);

    Serial.print("Target: ");
    Serial.print(targetRPM, 0);

    Serial.print(" RPM");

    Serial.print(" | Actual: ");
    Serial.print(actualRPM, 1);

    Serial.print(" RPM");

    Serial.print(" | Pulses: ");
    Serial.print(pulsesSincePrint);

    Serial.print(" | PID Output: ");
    Serial.print(pidOutput, 0);

    Serial.print(" | Motor Power: ");
    Serial.print(motorPower, 0);

    Serial.print(" | Light: ");
    Serial.print(lightValue);

    Serial.print(" | Threshold: ");
    Serial.println(lightThreshold);

    pulsesSincePrint = 0;
  }
}