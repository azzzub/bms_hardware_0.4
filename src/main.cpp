#include <Arduino.h>
#include <HardwareSerial.h>
#include "bq769x0.h" // Library for Texas Instruments bq76920 battery management IC

#define BMS_ALERT_PIN PB_12 // attached to interrupt INT0
#define BMS_BOOT_PIN PC_13  // connected to TS1 input
#define BMS_I2C_ADDRESS 0x18

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
float nominalCapacity_Ah = 45;
size_t numOcvPoints = sizeof(ocv_lfp) / sizeof(float);
float *ocv = ocv_lfp;

//*********

void resetSOC(void);

void setup()
{

  Serial.begin(19200);
  Serial3.begin(9600);
  pinMode(LED, OUTPUT);

  // while (1)
  // {
  digitalWrite(LED, HIGH);
  // delay(100);
  // digitalWrite(LED, LOW);
  // delay(100);
  // digitalWrite(LED, HIGH);
  // delay(100);
  // digitalWrite(LED, LOW);
  // delay(100);
  // Serial.println("Test");
  // }

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
  Serial.println("Cell_1,Cell_2,Cell_3,Cell_4,Cell_5,Total,Temp,Current");
  BMS.update();
  averageCellVoltage = (BMS.getBatteryVoltage() / 4.0) / 1000.0f;
  resetSOC();
  delay(1000);
}

bool loadStatus = true;
bool chargeStatus = true;

void loop()
{
  if (Serial3.available())
  {
    String inputSerial = Serial3.readStringUntil(';');
    if (inputSerial == "LD_0")
    {
      loadStatus = false;
    }
    else if (inputSerial == "LD_1")
    {
      loadStatus = true;
    }
    else if (inputSerial == "CG_1")
    {
      chargeStatus = true;
    }
    else if (inputSerial == "CG_0")
    {
      chargeStatus = false;
    }
  }

  if (loadStatus)
  {
    BMS.enableDischarging();
  }
  else
  {
    BMS.disableDischarging();
  }

  if (chargeStatus)
  {
    BMS.enableCharging();
  }
  else
  {
    BMS.disableCharging();
  }

  if (millis() - currentMillis >= 900)
  {
    digitalWrite(LED, LOW);
    delay(100);
    digitalWrite(LED, HIGH);
    BMS.update();
    BMS.enableDischarging();
    BMS.enableCharging();

    for (int i = 1; i <= 5; i++)
    {
      int cellVoltage = BMS.getCellVoltage(i);
      if (cellVoltage <= 50)
      {
        continue;
      }
      if (currentDelta == 0)
      {

        currentDelta = cellVoltage;
      }
      else
      {
        delta += abs(currentDelta - cellVoltage);
        currentDelta = cellVoltage;
      }
      cellVoltages[batteryCounter] = cellVoltage;
      batteryCounter += 1;
    }

    Serial3.print(batteryCounter);
    Serial3.print(",");

    for (int i = 0; i < batteryCounter; i++)
    {
      Serial3.print(cellVoltages[i] / 1000.0f, 3);
      Serial3.print(",");
    }

    batteryCounter = 0;

    delta = delta / 4;
    currentDelta = 0;

    float totalBatteryVoltage = BMS.getBatteryVoltage() / 1000.0f;

    Serial3.print(totalBatteryVoltage, 3);
    Serial3.print(",");
    float batteryCurrent = BMS.getBatteryCurrent() / 1000.0f;
    columbCounter_mAs += batteryCurrent * 1000;
    soc = columbCounter_mAs / (nominalCapacity_Ah * 3.6e4F);
    Serial3.print(soc, 2);
    Serial3.print("%,");
    Serial3.print(delta / 1000.0f, 3);
    Serial3.print(",");
    Serial3.print(BMS.getMinCellVoltage() / 1000.0f, 3);
    Serial3.print(",");
    Serial3.print(BMS.getMaxCellVoltage() / 1000.0f, 3);
    Serial3.print(",");
    Serial3.print(BMS.getTemperatureDegC());
    Serial3.print(",");

    Serial3.print(abs(batteryCurrent), 2);
    Serial3.print(",");
    float power = abs(batteryCurrent * totalBatteryVoltage);
    Serial3.print(power, 2);
    Serial3.print(",");
    Serial3.println(batteryCurrent > 0 ? 1 : batteryCurrent == 0 ? 0
                                                                 : -1);

    currentMillis = millis();
  }
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