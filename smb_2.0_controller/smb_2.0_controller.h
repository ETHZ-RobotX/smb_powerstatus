#ifndef SMB_2_CONTROLLER_H__
#define SMB_2_CONTROLLER_H__

#define USE_ROS

#ifdef USE_ROS
#define USE_USBCON          // Use native USB Serial as COM port for ROS
#endif

/************* DEFINES *************/

// Timer 
#define TIMER_MILLIS    1000
bool timer_flag = false;

// ADS1015 and ADC conversion defines
#define ADS1015_LSB_VOLTAGE           0.001
#define ADC_RESISTOR_DIVIDER_RATIO    ((6.8 + 91)/6.8)

// Battery define
#define BATTERY_LOW_INDICATION      3.2 * 7 
#define POWER_PRESENT_VOLTAGE       18       // minimum voltage, to be sure that a battery is plugged in
#define BATTERY_MAXIMUM             29.4
#define BATTERY_MINUMUM             19.8
#define POWER_PRESENT(voltage)    voltage > POWER_PRESENT_VOLTAGE

// LTC2944
#define LTC2944_RESISTOR            1 //milliohms


// Define struct for the voltages
struct data_struct
{
    // Voltages from ADS1015 
    float v_acdc;
    float v_bat1;
    float v_bat2;
    // Voltages and current from LTC2944
    float v_out;
    float c_out;
    float temp;
    // Battery use
    bool acdc_use;
    bool bat1_use;
    bool bat2_use;
    // Battery valid
    bool acdc_val;
    bool bat1_val;
    bool bat2_val;

};

/************* FUNCTIONS DEFINES *************/

#ifdef USE_ROS
// Publish the data to ROS
void publishROS();
#else
// Print the data int the seril console
void publishSerial();
#endif

// Set timer flag
bool setTimerFlag();

// Read sensors data
void readSensorsData();

// Convert voltage to percentage
float mapVoltageToPercentage(float voltage);

//Set the leds
void setLeds();

void powerSourceUsed();


#endif