#include <Arduino.h>
#include <HardwareSerial.h>
#include "bq769x0.h" // Library for Texas Instruments bq76920 battery management IC

#include <BlynkSimpleSerialBLE.h>

#define BMS_ALERT_PIN PB_12 // attached to interrupt INT0
#define BMS_BOOT_PIN PC_13  // connected to TS1 input
#define BMS_I2C_ADDRESS 0x18

char auth[] = "IFAhU9LxxWB6t5DuaYKIkX26-JGQ-1xV";

#define LED PB_13

#define DEBUG

bq769x0 BMS(bq76920, BMS_I2C_ADDRESS); // battery management system object
HardwareSerial Serial3(PB_11, PB_10);

int currentMillis = 0;
int count = 0;
int batteryCounter = 0;
float noob_soc = 0;
float currentDelta = 0;
float delta = 0;

int cellVoltages[255];

//*********
float ocv_lfp[] = { // 100, 95, ..., 0 %
    3.392, 3.314, 3.309, 3.308, 3.304, 3.296, 3.283, 3.275, 3.271, 3.268, 3.265,
    3.264, 3.262, 3.252, 3.240, 3.226, 3.213, 3.190, 3.177, 3.132, 2.833};

uint32_t columbCounter_mAs;
float soc;
float averageCellVoltage;
int percent = -1;
float nominalCapacity_Ah = 40;
size_t numOcvPoints = sizeof(ocv_lfp) / sizeof(float);
float *ocv = ocv_lfp;

//*********

void resetSOC(void);
void sendSensor(void);

BlynkTimer timer;

void setup()
{
  Serial.begin(19200);
  Serial3.begin(9600);
  pinMode(LED, OUTPUT);

  while (BMS.begin(BMS_ALERT_PIN, BMS_BOOT_PIN) == 1)
  {
    digitalWrite(LED, HIGH);
    delay(500);
    digitalWrite(LED, LOW);
    delay(500);
    count++;
    if (count > 10)
    {
      while (1)
      {
        digitalWrite(LED, HIGH);
        delay(100);
        digitalWrite(LED, LOW);
        delay(100);
      }
    }
  }

  digitalWrite(LED, HIGH);

  BMS.setTemperatureLimits(-20, 45, 0, 45);
  BMS.setShuntResistorValue(1);
  BMS.setShortCircuitProtection(200000, 1000);        // delay in us
  BMS.setOvercurrentChargeProtection(8000, 200);      // delay in ms
  BMS.setOvercurrentDischargeProtection(100000, 320); // delay in ms
  BMS.setCellUndervoltageProtection(2500, 2);         // delay in s
  BMS.setCellOvervoltageProtection(3650, 2);          // delay in s

  BMS.setBalancingThresholds(0, 3450, 20); // minIdleTime_min, minCellV_mV, maxVoltageDiff_mV
  BMS.setIdleCurrentThreshold(300);
  BMS.enableAutoBalancing();
  BMS.update();
  averageCellVoltage = (BMS.getBatteryVoltage() / 4.0) / 1000.0f;
  resetSOC();
  delay(1000);
  Blynk.begin(Serial3, auth);

  digitalWrite(LED, HIGH);
  timer.setInterval(1000L, sendSensor);
}

bool loadStatus = true;
bool chargeStatus = true;

void loop()
{
  Blynk.run();
  timer.run();
}

WidgetLED led(V0);

void sendSensor()
{
  digitalWrite(LED, LOW);
  led.on();
  delay(100);
  digitalWrite(LED, HIGH);
  led.off();
  BMS.update();
  BMS.enableDischarging();
  BMS.enableCharging();
  int cell1 = BMS.getCellVoltage(1);
  int cell2 = BMS.getCellVoltage(2);
  int cell3 = BMS.getCellVoltage(3);
  int cell4 = BMS.getCellVoltage(5);
  float totalBatteryVoltage = BMS.getBatteryVoltage() / 1000.0f;
  float batteryCurrent = BMS.getBatteryCurrent() / 1000.0f;
  columbCounter_mAs += batteryCurrent * 1000;
  soc = columbCounter_mAs / (nominalCapacity_Ah * 3.6e4F);
  float power = batteryCurrent * totalBatteryVoltage;

  Blynk.virtualWrite(V1, String(cell1 / 1000.0f, 3));
  Blynk.virtualWrite(V2, String(cell2 / 1000.0f, 3));
  Blynk.virtualWrite(V3, String(cell3 / 1000.0f, 3));
  Blynk.virtualWrite(V4, String(cell4 / 1000.0f, 3));
  Blynk.virtualWrite(V5, String(totalBatteryVoltage, 3));
  Blynk.virtualWrite(V6, String(batteryCurrent, 3));
  Blynk.virtualWrite(V7, String(soc, 3));
  Blynk.virtualWrite(V8, String(BMS.getMinCellVoltage() / 1000.0f, 3));
  Blynk.virtualWrite(V9, String(BMS.getMaxCellVoltage() / 1000.0f, 3));
  Blynk.virtualWrite(V10, String(BMS.getTemperatureDegC(), 2));
  Blynk.virtualWrite(V11, String(power, 3));
}

void resetSOC()
{
  if (percent <= 100 && percent >= 0)
  {
    columbCounter_mAs = nominalCapacity_Ah * 3.6e4F * percent;
  }
  else
  {
    columbCounter_mAs = 0;
    for (unsigned int i = 0; i < numOcvPoints; i++)
    {
      if (ocv[i] <= averageCellVoltage)
      {
        if (i == 0)
        {
          columbCounter_mAs = nominalCapacity_Ah * 3.6e6F;
        }
        else
        {
          columbCounter_mAs = nominalCapacity_Ah * 3.6e6F / (numOcvPoints - 1.0) * (numOcvPoints - 1.0 - i + (averageCellVoltage - ocv[i]) / (ocv[i - 1] - ocv[i]));
        }
        return;
      }
    }
  }
}