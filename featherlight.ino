#include <Wire.h>

#include <Adafruit_NeoPixel.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#define NEOPIXEL_PIN 10

const uint8_t s_numPixels = 16;

/**
 * Radial positions of the neopixels as unit quaternions.
 * This data is calculated from the PCB component placement output.
 *
 * Pixel 0 is over the USB jack, and the numbering continues
 * clockwise around the ring.
 */
const imu::Quaternion s_initialPixelPositions[s_numPixels] =
{
    imu::Quaternion(0.0, -1.00000000,  0.00000000, 0.0),
    imu::Quaternion(0.0, -0.92387970,  0.38268302, 0.0),
    imu::Quaternion(0.0, -0.70710678,  0.70710678, 0.0),
    imu::Quaternion(0.0, -0.38268302,  0.92387970, 0.0),
    imu::Quaternion(0.0,  0.00000000,  1.00000000, 0.0),
    imu::Quaternion(0.0,  0.38315683,  0.92368330, 0.0),
    imu::Quaternion(0.0,  0.70710678,  0.70710678, 0.0),
    imu::Quaternion(0.0,  0.92387970,  0.38268302, 0.0),
    imu::Quaternion(0.0,  1.00000000,  0.00000000, 0.0),
    imu::Quaternion(0.0,  0.92387970, -0.38268302, 0.0),
    imu::Quaternion(0.0,  0.70710678, -0.70710678, 0.0),
    imu::Quaternion(0.0,  0.38268302, -0.92387970, 0.0),
    imu::Quaternion(0.0,  0.00000000, -1.00000000, 0.0),
    imu::Quaternion(0.0, -0.38268302, -0.92387970, 0.0),
    imu::Quaternion(0.0, -0.70710678, -0.70710678, 0.0),
    imu::Quaternion(0.0, -0.92387970, -0.38268302, 0.0)
};

imu::Quaternion g_currentPixelPositions[s_numPixels];

/**
 * Initial radial positions of the three color primaries
 *
 * The representation is a quaternion pointing in the direction of
 * maximum brightness for each primary. the contribution to each pixel is
 * determined by a dot product calculation with that pixel's
 * updated position quaternion.
 */
const imu::Quaternion g_initialRGBPositions[3] =
{
    // Green
    imu::Quaternion(0.0, 0.0,  1.0, 0.0),
    // Red
    imu::Quaternion(0.0, 0.0,  0.0, 1.0),
    // Blue
    imu::Quaternion(0.0, 1.0,  0.0, 0.0)
};

/* Array index constants for the color primaries.
 * Blame neopixels for the wonky ordering.
 */
const uint8_t g_GREEN = 0;
const uint8_t g_RED   = 1;
const uint8_t g_BLUE  = 2;

Adafruit_NeoPixel pixels = Adafruit_NeoPixel(s_numPixels, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

Adafruit_BNO055 bno28 = Adafruit_BNO055(-1, 0x28);
Adafruit_BNO055 bno29 = Adafruit_BNO055(-1, 0x29);

Adafruit_BNO055* bno055 = NULL;

void setup()
{
    Serial.begin(9600);

  /* Initialise the sensor */
  if(bno28.begin())
  {
    bno055 = &bno28;
  }
  else if(bno29.begin())
  {
    bno055 = &bno29;
  }
  else
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    SerialUSB.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(true)
    {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(250);
      digitalWrite(LED_BUILTIN, LOW);
      delay(250);
    }
  }

    delay(1000);

    bno055->setExtCrystalUse(true);

    pixels.begin();
    pixels.setBrightness(16);
    pixels.show();
}

void loop()
{
    imu::Quaternion imuRotation = bno055->getQuat();
    imu::Quaternion imuConjugate = imuRotation.conjugate();

    /*
     * Generate updated positions for this tick.
     *
     * General quaternion rotation equation V' = Q * V * Q^-1
     *
     * The BNO055 outputs Q directly, so save that and its conjugate
     * and crank through the math.
     */
    for(uint8_t i = 0; i < s_numPixels; ++i)
    {
        uint8_t redSubpixel = 0;
        uint8_t greenSubpixel = 0;
        uint8_t blueSubpixel = 0;
        g_currentPixelPositions[i] = imuRotation * s_initialPixelPositions[i] * imuConjugate;

        redSubpixel = calculateSubpixel(g_currentPixelPositions[i], g_initialRGBPositions[g_RED]);
        greenSubpixel = calculateSubpixel(g_currentPixelPositions[i], g_initialRGBPositions[g_GREEN]);
        blueSubpixel = calculateSubpixel(g_currentPixelPositions[i], g_initialRGBPositions[g_BLUE]);

        pixels.setPixelColor(i, redSubpixel, greenSubpixel, blueSubpixel);
//        pixels.setPixelColor(15-i, redSubpixel, greenSubpixel, blueSubpixel);
    }

//    debugQuaternion(imuRotation);
//    displayCalStatus();
//    Serial.println("");

    pixels.show();
    delay(100); // FIXME this is in milliseconds, so the IMU updates every 10.  Getting this down would be nice.
}

uint8_t calculateSubpixel(const imu::Quaternion& p_pixel, const imu::Quaternion& p_subpixelPrimary)
{
    imu::Vector<3> pixelProjection(p_pixel.x(), p_pixel.y(), p_pixel.z());
    imu::Vector<3> primaryProjection(p_subpixelPrimary.x(), p_subpixelPrimary.y(), p_subpixelPrimary.z());

    double rawSubpixel = pixelProjection.dot(primaryProjection) / 2 + 0.5;
    return static_cast<uint8_t>(255.0 * rawSubpixel);
}

/**
 *  Display sensor calibration status
 */
void debugQuaternion(const imu::Quaternion& p_quat)
{
    Serial.print("qW: ");
    Serial.print(p_quat.w(), 4);
    Serial.print(" qX: ");
    Serial.print(p_quat.y(), 4);
    Serial.print(" qY: ");
    Serial.print(p_quat.x(), 4);
    Serial.print(" qZ: ");
    Serial.print(p_quat.z(), 4);
    Serial.print("\t");
}

/**
 *  Display sensor calibration status
 */
void displayCalStatus(void)
{
    /* Get the four calibration values (0..3) */
    /* Any sensor data reporting 0 should be ignored, */
    /* 3 means 'fully calibrated" */
    uint8_t system, gyro, accel, mag;
    system = gyro = accel = mag = 0;
    bno055->getCalibration(&system, &gyro, &accel, &mag);

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
