#include <Wire.h>

#include <Adafruit_NeoPixel.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#define NEOPIXEL_PIN 6
#define IMU_SAMPLE_DELAY 100

Adafruit_NeoPixel ring = Adafruit_NeoPixel(8*4, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);
Adafruit_BNO055 featherlight_imu = Adafruit_BNO055(55);

uint16_t rainbow_state = 0;

void setup()
{
    Serial.begin(9600);
//    if(!featherlight_imu.begin(Adafruit_BNO055::OPERATION_MODE_NDOF, 0x06, 0x00))
    if(!featherlight_imu.begin())
    {
        Serial.print("Could not initialize IMU");
        while(1);
    }
    
    delay(1000);
    
    featherlight_imu.setExtCrystalUse(true);
    
    ring.begin();
    ring.setBrightness(16);
    ring.show();
}

void loop()
{
    //rainbowTick();
    delay(500);
    
     imu::Quaternion quat = featherlight_imu.getQuat();
     Serial.print("qW: ");
     Serial.print(quat.w(), 4);
     Serial.print(" qX: ");
     Serial.print(quat.y(), 4);
     Serial.print(" qY: ");
     Serial.print(quat.x(), 4);
     Serial.print(" qZ: ");
     Serial.print(quat.z(), 4);
     Serial.print("\t");
    
    /* Get a new sensor event */
    sensors_event_t event;
    featherlight_imu.getEvent(&event);

    /* Display the floating point data */
    Serial.print("X: ");
    Serial.print(event.orientation.x, 4);
    Serial.print("\tY: ");
    Serial.print(event.orientation.y, 4);
    Serial.print("\tZ: ");
    Serial.print(event.orientation.z, 4);

    displayCalStatus();
    Serial.println("");
}

void rainbowTick()
{
    uint16_t i;
    
    ++rainbow_state;
    if(rainbow_state == 256)
    {
        rainbow_state = 0;
    }
        
    for(i=0; i<ring.numPixels(); i++)
    {
        ring.setPixelColor(i, Wheel((i+rainbow_state) & 255));
    }
    ring.show();
}

uint32_t Wheel(byte WheelPos)
{
    WheelPos = 255 - WheelPos;
    if(WheelPos < 85)
    {
        return ring.Color(255 - WheelPos * 3, 0, WheelPos * 3);
    }
    if(WheelPos < 170)
    {
        WheelPos -= 85;
        return ring.Color(0, WheelPos * 3, 255 - WheelPos * 3);
    }
    WheelPos -= 170;
    return ring.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
}


/**************************************************************************/
/*
    Display some basic info about the sensor status
*/
/**************************************************************************/
void displaySensorStatus(void)
{
    /* Get the system status values (mostly for debugging purposes) */
    uint8_t system_status, self_test_results, system_error;
    system_status = self_test_results = system_error = 0;
    featherlight_imu.getSystemStatus(&system_status, &self_test_results, &system_error);

    /* Display the results in the Serial Monitor */
    Serial.println("");
    Serial.print("System Status: 0x");
    Serial.println(system_status, HEX);
    Serial.print("Self Test:     0x");
    Serial.println(self_test_results, HEX);
    Serial.print("System Error:  0x");
    Serial.println(system_error, HEX);
    Serial.println("");
    delay(500);
}

/**************************************************************************/
/*
    Display sensor calibration status
*/
/**************************************************************************/
void displayCalStatus(void)
{
    /* Get the four calibration values (0..3) */
    /* Any sensor data reporting 0 should be ignored, */
    /* 3 means 'fully calibrated" */
    uint8_t system, gyro, accel, mag;
    system = gyro = accel = mag = 0;
    featherlight_imu.getCalibration(&system, &gyro, &accel, &mag);

    /* The data should be ignored until the system calibration is > 0 */
    Serial.print("\t");
    if (!system)
    {
        Serial.print("! ");
    }

    /* Display the individual values */
    Serial.print("Sys:");
    Serial.print(system, DEC);
    Serial.print(" G:");
    Serial.print(gyro, DEC);
    Serial.print(" A:");
    Serial.print(accel, DEC);
    Serial.print(" M:");
    Serial.print(mag, DEC);
}
