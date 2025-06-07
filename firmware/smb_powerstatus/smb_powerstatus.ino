/************* INCLUDES *************/
#include <stdio.h>
#include "smb_powerstatus.h"
#include <arduino-timer.h>
#include <Adafruit_ADS1X15.h>
#include "LTC3219.h"
#include "TCA9554.hpp"
#include <Wire.h>

/************* DEFINES *************/
// LED
#define LED_ACDC_GREEN            LTC3219_LED1_REGISTER
#define LED_ACDC_RED              LTC3219_LED2_REGISTER
#define LED_BAT1_GREEN            LTC3219_LED3_REGISTER
#define LED_BAT1_RED              LTC3219_LED4_REGISTER
#define LED_BAT2_GREEN            LTC3219_LED5_REGISTER
#define LED_BAT2_RED              LTC3219_LED6_REGISTER

/************* GLOBAL VARIABLES *************/
auto timer = timer_create_default();

// Voltages
Adafruit_ADS1015 ads1015;
power_status_struct data;

// LED
LTC3219 ltc3219;

// Battery Valid
TCA9554 tca9554;
uint8_t tca9554_value = 0;

/************* SETUP *************/
void setup(){


    // Begin Wire
    Wire.begin();

    //Initialize Serial communication
    Serial.begin(115200);
    while(!Serial) {
      ; // wait for serial port to connect.
    }

    // Initialize timer
    timer.every(TIMER_MILLIS, setTimerFlag);


    //Setup ADS1015
    //ads1015.setGain(GAIN_TWO);
    if (!ads1015.begin()) {
      Serial.println("Failed to initialize ADS1015!");
    } else {
      Serial.println("ADS1015 initialized.");
    }

    //Setup LTC3219
    ltc3219.begin();

    //Setup TCA9554
    tca9554.begin();
#ifdef CSVOUTPUT
    Serial.println("V_ACDC;V_Bat1;V_Bat2;I;ActiveInput");
#endif
}

/************* LOOP *************/
void loop(){
    timer.tick();

    readSensorsData();

    powerSourceUsed();

    setLeds();

    // Send data wenn timer is triggered
    if(timer_flag){
        publishSerial();
        timer_flag = false;
    }
}


void publishSerial(){
#ifdef CSVOUTPUT
    Serial.print(data.v_acdc);
    Serial.print("V;");
    Serial.print(data.v_bat1);
    Serial.print("V;");
    Serial.print(data.v_bat2);
    Serial.print("V;");
    Serial.print(data.i_out);
    Serial.print("A;");
    if (data.acdc_use) Serial.print("ACDC");
    else if (data.bat1_use) Serial.print("BAT1");
    else if (data.bat2_use) Serial.print("BAT2");
    else Serial.print("none");
    Serial.println();
#else
    Serial.println();
    Serial.println("Arduino measurements:");
    Serial.print("ACDC: ");
    Serial.print(data.v_acdc);
    Serial.print("V");
    Serial.print("    ");
    Serial.print("Bat 1: ");
    Serial.print(data.v_bat1);
    Serial.print("V");
    Serial.print("    ");
    Serial.print("Bat 2: ");
    Serial.print(data.v_bat2);
    Serial.print("V");
    Serial.print("    ");

    Serial.println();
    Serial.println();
    Serial.println("Current measurements (TLI4971 on ADC port 3):");
    Serial.print("ADC port3: ");
    Serial.print(data.v_aout);
    Serial.print("V    -->  ");
    Serial.print(data.i_out);
    Serial.print("A     ");
    Serial.println();
    Serial.println();

    Serial.print("Active input: ");
    if (data.acdc_use) Serial.print("AC DC (X1)");
    if (data.bat1_use) Serial.print("BAT 1 (X2)");
    if (data.bat2_use) Serial.print("BAT 2 (X3)");
    Serial.println();
    Serial.println();
    Serial.print("TCA9554 data:");
    Serial.print(tca9554_value);
    Serial.println();
    Serial.println();

#endif // CSVOUTPUT
}

bool setTimerFlag(void *){
    timer_flag = true;

    return true;
}

void readSensorsData(){
    // Read analog inputs
    int16_t adc0, adc1, adc2, adc3;
    adc0 = ads1015.readADC_SingleEnded(0u);
    delay(10);
    adc1 = ads1015.readADC_SingleEnded(1u);
    delay(10);
    adc2 = ads1015.readADC_SingleEnded(2u);
    delay(10);
    adc3 = ads1015.readADC_SingleEnded(3u);
    data.v_acdc = ads1015.computeVolts(adc0) * ADC_RESISTOR_DIVIDER_RATIO;
    data.v_bat1 = ads1015.computeVolts(adc1) * ADC_RESISTOR_DIVIDER_RATIO;
    data.v_bat2 = ads1015.computeVolts(adc2) * ADC_RESISTOR_DIVIDER_RATIO;

    // Convert data from TLI4971
    data.v_aout = ads1015.computeVolts(adc3);
    data.i_out = (data.v_aout-TLI4971_VREF) / TLI4971_SENSITIVITY;

    //Read from tca9554
    tca9554.readInputs(&tca9554_value);
}

void powerSourceUsed(){
    bool power_valid[3] = {0};
    tca9554.getBatteryValid(power_valid);
    data.acdc_val = power_valid[0];
    data.bat1_val = power_valid[1];
    data.bat2_val = power_valid[2];
    if(power_valid[0]){
        data.acdc_use = true;
        data.bat1_use = false;
        data.bat2_use = false;
    } else if (power_valid[1] && !power_valid[0])
    {
        data.acdc_use = false;
        data.bat1_use = true;
        data.bat2_use = false;        
    } else if (power_valid[2] && !power_valid[0] && !power_valid[1])
    {
        data.acdc_use = false;
        data.bat1_use = false;
        data.bat2_use = true;
    } else {
        data.acdc_use = false;
        data.bat1_use = false;
        data.bat2_use = false;
    }
}

float mapVoltageToPercentage(float voltage){
    if (voltage < BATTERY_MINUMUM)
    {
        return 0.0;
    }
    if (voltage > BATTERY_MAXIMUM){
        return 1.0;
    }

    return (voltage - BATTERY_MINUMUM) / (BATTERY_MAXIMUM - BATTERY_MINUMUM);
}

void setLeds()
{
  if(data.v_acdc >= BATTERY_LOW_INDICATION)
  {
      ltc3219.turnOnLED(LED_ACDC_GREEN);
      ltc3219.turnOffLED(LED_ACDC_RED); 
  }
  else
  {
    if((data.v_acdc < BATTERY_LOW_INDICATION) && (data.acdc_val))
    {
        ltc3219.setLEDBlink(LED_ACDC_GREEN);
        ltc3219.turnOffLED(LED_ACDC_RED);
    }else if((data.v_acdc >= POWER_PRESENT_VOLTAGE) && !(data.acdc_val))
    {
        ltc3219.turnOffLED(LED_ACDC_GREEN);
        ltc3219.turnOnLED(LED_ACDC_RED); 
    }
    else
    {
        ltc3219.turnOffLED(LED_ACDC_GREEN);
        ltc3219.turnOffLED(LED_ACDC_RED);
    }
  }
  if(data.v_bat1 >= BATTERY_LOW_INDICATION)
  {
      ltc3219.turnOnLED(LED_BAT1_GREEN);
      ltc3219.turnOffLED(LED_BAT1_RED);  
  }
  else
  {
    if((data.v_bat1 < BATTERY_LOW_INDICATION) && (data.bat1_val))
    {
        ltc3219.setLEDBlink(LED_BAT1_GREEN);
        ltc3219.turnOffLED(LED_BAT1_RED);
    }else if((data.v_bat1 >= POWER_PRESENT_VOLTAGE) && !(data.bat1_val))
    {
        ltc3219.turnOffLED(LED_BAT1_GREEN);
        ltc3219.turnOnLED(LED_BAT1_RED);
    }
    else
    {
        ltc3219.turnOffLED(LED_BAT1_GREEN);
        ltc3219.turnOffLED(LED_BAT1_RED);
    }
  }
  if(data.v_bat2 >= BATTERY_LOW_INDICATION)
  {
      ltc3219.turnOnLED(LED_BAT2_GREEN);
      ltc3219.turnOffLED(LED_BAT2_RED);    
  }
  else
  {
    if((data.v_bat2 < BATTERY_LOW_INDICATION) && (data.bat2_val))
    {
        ltc3219.setLEDBlink(LED_BAT2_GREEN);
        ltc3219.turnOffLED(LED_BAT2_RED);
    }else if((data.v_bat2 >= POWER_PRESENT_VOLTAGE) && !(data.bat2_val))
    {
        ltc3219.turnOffLED(LED_BAT2_GREEN);
        ltc3219.turnOnLED(LED_BAT2_RED);        
    }
    else
    {
        ltc3219.turnOffLED(LED_BAT2_GREEN);
        ltc3219.turnOffLED(LED_BAT2_RED);
    }
  }
}