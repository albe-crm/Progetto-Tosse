#include <Wire.h>
#include "SparkFun_BMI270_Arduino_Library.h"
#include <Adafruit_TinyUSB.h>

#define TIME_INTGR 100
#define SOGLIA_TOSSE 0.35

// Create a new sensor object
BMI270 imu;

// I2C address selection
uint8_t i2cAddress = BMI2_I2C_PRIM_ADDR; // 0x68
//uint8_t i2cAddress = BMI2_I2C_SEC_ADDR; // 0x69
double a_x, a_y, a_z, d_x, d_y, d_z, delta, intgr_z = 0;
long t_old, timer = 0;
bool tosse = 0;

void setup()
{
    // Start serial
    Serial.begin(115200);
    Serial.println("BMI270 Example 1 - Basic Readings I2C");

    // Initialize the I2C library
    Wire.begin();

    // Check if sensor is connected and initialize
    // Address is optional (defaults to 0x68)
    unsigned long t0 = millis();
    while (!Serial && millis() - t0 < 3000) {
    // codice prof, do nothing
    }
    while(imu.beginI2C(i2cAddress) != BMI2_OK)
    {
        // Not connected, inform user
        Serial.println("Error: BMI270 not connected, check wiring and I2C address!");
        

        // Wait a bit to see if connection is established
        delay(1000);
    }

    Serial.println("BMI270 connected!");
}

void loop()
{
    // Get measurements from the sensor. This must be called before accessing
    // the sensor data, otherwise it will never update
    
    imu.getSensorData();

    delta = millis() - t_old;
    t_old = millis();
    // calcolo intervallo di tempo tra acquisizione attuale e precedente (al tempo t_old)

    d_z = (imu.data.accelZ - a_z)/ delta;
    a_z = imu.data.accelZ;
    // calcolo derivata accelerazione con la differenza tra accelerazione attuale e precedente (a_z)

    intgr_z += abs(d_z);
    // calcolo integrale come somma delle derivate (la somma sara' azzerata dopo un intervallo di tempo TIME_INTGR)

    if(millis() - timer >= TIME_INTGR){
        // verifico tosse se integrale supera un certo valore soglia SOGLIA_TOSSE (preso sperimentalmente)
        if(intgr_z > SOGLIA_TOSSE){
            tosse = 1;
        }
        else tosse = 0;

        //commenta questi serial.print se vuoi usare serial plotter
        // Serial.print("Integrale = ");
        // Serial.print(intgr_z,5);
        // Serial.print(" | tosse = ");

        Serial.println(intgr_z);
        // azzero integrale e ri-inizializzo il timer
        timer = millis();
        intgr_z = 0;
    }



    

    // Print 50x per second
    //delay(250);
}