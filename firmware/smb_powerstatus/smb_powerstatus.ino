/************* INCLUDES *************/
#include <stdio.h>
#include "smb_powerstatus.h"
#include <ros.h>
#include <smb_powerstatus/SMBPowerStatus.h>
#include "arduino-timer.h"
#include <Adafruit_ADS1X15.h>
#include "LTC2944.h"
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
#ifdef USE_ROS
// Custom message SMBPowerStatus
smb_powerstatus::SMBPowerStatus smb_power_msg;

// Nodehandle and publisher
ros::NodeHandle nh;
ros::Publisher smb_power_pub("/smb_powerstatus/payload", &smb_power_msg);
#endif

auto timer = timer_create_default();


// Voltages
Adafruit_ADS1015 ads1015;
LTC2944 ltc2944(LTC2944_RESISTOR);
data_struct data;

// LED
LTC3219 ltc3219;

// Battery Valid
TCA9554 tca9554;
uint8_t tca9554_value = 0;

/************* SETUP *************/
void setup(){


// Begin Wire
Wire.begin();

#ifdef USE_ROS
    // Initialize ROS publisher
    nh.getHardware()->setBaud(115200);
    nh.initNode();
    nh.advertise(smb_power_pub);
#else
    //Initialize Serial communication
    Serial.begin(115200);
#endif

    // Initialize timer
    timer.every(TIMER_MILLIS, setTimerFlag);


    //Setup ADS1015
    ads1015.setGain(GAIN_TWO);
    ads1015.begin();

    while (ltc2944.begin() == false) {
        Serial.println("Failed to detect LTC2944!");
    delay(5000);
    }

    //Setup LTC2944
    ltc2944.setPrescalerM(256);
    ltc2944.setADCMode(ADC_MODE_SLEEP);
    ltc2944.startMeasurement();

    //Setup LTC3219
    ltc3219.begin();

    //Setup TCA9554
    tca9554.begin();

}

/************* LOOP *************/
void loop(){
    timer.tick();

    readSensorsData();

    powerSourceUsed();

    setLeds();

    // Send data wenn timer is triggered
    if(timer_flag){
        #ifdef USE_ROS
        publishROS();
        #else
        publishSerial();
        #endif
        timer_flag = false;
    }
}

#ifdef USE_ROS
void publishROS(){
    // ACDC data
    smb_power_msg.power_supply_present = POWER_PRESENT(data.v_acdc);
    smb_power_msg.power_supply_voltage = data.v_acdc;

    // Output voltage and current
    smb_power_msg.output_voltage = data.v_out;
    smb_power_msg.output_current = data.c_out;

    // BATTERY 1 data
    smb_power_msg.battery_1.present = POWER_PRESENT(data.v_bat1);
    smb_power_msg.battery_1.voltage = data.v_bat1;
    smb_power_msg.battery_1.percentage = mapVoltageToPercentage(data.v_bat1);
    if(data.bat1_use){
        smb_power_msg.battery_1.power_supply_status = smb_power_msg.battery_1.POWER_SUPPLY_STATUS_DISCHARGING;
        smb_power_msg.battery_1.current = data.c_out;
    }else{
        smb_power_msg.battery_1.power_supply_status = smb_power_msg.battery_1.POWER_SUPPLY_STATUS_NOT_CHARGING;
        smb_power_msg.battery_1.current = 0;
    }

    // BATTERY 2 data
    smb_power_msg.battery_2.present = POWER_PRESENT(data.v_bat2);
    smb_power_msg.battery_2.voltage = data.v_bat2;
    smb_power_msg.battery_2.percentage = mapVoltageToPercentage(data.v_bat2);
    if(data.bat2_use){
        smb_power_msg.battery_2.power_supply_status = smb_power_msg.battery_2.POWER_SUPPLY_STATUS_DISCHARGING;
        smb_power_msg.battery_2.current = data.c_out;
    }else{
        smb_power_msg.battery_2.power_supply_status = smb_power_msg.battery_2.POWER_SUPPLY_STATUS_NOT_CHARGING;
        smb_power_msg.battery_2.current = 0;
    }


    // Publish the data
    smb_power_pub.publish(&smb_power_msg);
    nh.spinOnce();
}
#else
void publishSerial(){

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
    Serial.println("LTC2944 measurements:");
    Serial.print("Output: ");
    Serial.print(data.v_out);
    Serial.print("V  ");
    Serial.print(data.c_out);
    Serial.print("A     ");
    Serial.print(data.temp);
    Serial.print("°C");
    Serial.println();

    Serial.print("Port     ");
    Serial.print(data.acdc_val);
    Serial.println();


}
#endif

bool setTimerFlag(void *){
    timer_flag = true;

    return true;
}

void readSensorsData(){
    // Read analog inputs
    int16_t adc0, adc1, adc2;
    adc0 = ads1015.readADC_SingleEnded(0);
    adc1 = ads1015.readADC_SingleEnded(1);
    adc2 = ads1015.readADC_SingleEnded(2);
    data.v_acdc = (float)adc0 * ADS1015_LSB_VOLTAGE * ADC_RESISTOR_DIVIDER_RATIO;
    data.v_bat1 = (float)adc1 * ADS1015_LSB_VOLTAGE * ADC_RESISTOR_DIVIDER_RATIO;
    data.v_bat2 = (float)adc2 * ADS1015_LSB_VOLTAGE * ADC_RESISTOR_DIVIDER_RATIO;

    // Read data from LTC2944
    data.v_out = ltc2944.getVoltage();
    data.c_out = ltc2944.getCurrent();
    data.temp = ltc2944.getTemperature();

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
    }else if (power_valid[1] && !power_valid[0])
    {
        data.acdc_use = false;
        data.bat1_use = true;
        data.bat2_use = false;        
    }else
    {
        data.acdc_use = false;
        data.bat1_use = false;
        data.bat2_use = true;
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