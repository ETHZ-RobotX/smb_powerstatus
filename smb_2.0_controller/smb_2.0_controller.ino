/************* INCLUDES *************/
#include <stdio.h>
#include "smb_2.0_controller.h"
#include <ros.h>
#include <smb_rviz_plugins/SMBPower.h>
#include "arduino-timer.h"
#include <Adafruit_ADS1X15.h>
#include "LTC2944.h"


/************* GLOBAL VARIABLES *************/
#ifdef USE_ROS
// Custom message SMBPower
smb_rviz_plugins::SMBPower smb_power_msg;

// Nodehandle and publisher
ros::NodeHandle nh;
ros::Publisher smb_power_pub("/smb_power/payload", &smb_power_msg);
#endif

auto timer = timer_create_default();


// Voltages
Adafruit_ADS1015 ads1015;
LTC2944 ltc2944(LTC2944_RESISTOR);
data_struct data;

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

    //Setup LTC2944
    ltc2944.setPrescalerM(256);
    ltc2944.setADCMode(ADC_MODE_SLEEP);
    ltc2944.startMeasurement();

}

/************* LOOP *************/
void loop(){
    timer.tick();

    readSensorsData();

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

    // BATTERY 1 data
    smb_power_msg.battery_1.present = POWER_PRESENT(data.v_bat1);
    smb_power_msg.battery_1.voltage = data.v_bat1;
    smb_power_msg.battery_1.percentage = mapVoltageToPercentage(data.v_bat1);
    smb_power_msg.battery_1.power_supply_status = smb_power_msg.battery_1.POWER_SUPPLY_STATUS_DISCHARGING; //TODO

    // BATTERY 2 data
    smb_power_msg.battery_2.present = POWER_PRESENT(data.v_bat2);
    smb_power_msg.battery_2.voltage = data.v_bat2;
    smb_power_msg.battery_2.percentage = mapVoltageToPercentage(data.v_bat2);
    smb_power_msg.battery_2.power_supply_status = smb_power_msg.battery_2.POWER_SUPPLY_STATUS_DISCHARGING; //TODO


    // Publish the data
    smb_power_pub.publish(&smb_power_msg);
    nh.spinOnce();
}
#else
void publishSerial(){

 

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


