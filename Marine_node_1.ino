#include <Wire.h>
#include <TinyGPS++.h>

#define MPU_ADDR 0x68

TinyGPSPlus gps;

// Global variables
int16_t ax, ay, az;
float tilt = 0;

float latitude = 0;
float longitude = 0;

//-------------------------------------------------------------

void initMPU()
{
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x6B);
    Wire.write(0x00);
    Wire.endTransmission(true);

    Serial2.println("MPU6050 Initialized");
}

//-------------------------------------------------------------

void readMPU()
{
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B);

    if (Wire.endTransmission(false) == 0)
    {
        Wire.requestFrom(MPU_ADDR, 6, true);

        if (Wire.available() >= 6)
        {
            ax = (Wire.read() << 8) | Wire.read();
            ay = (Wire.read() << 8) | Wire.read();
            az = (Wire.read() << 8) | Wire.read();

            tilt = atan2((float)ay, (float)az) * 180.0 / PI;
        }
    }
}

//-------------------------------------------------------------

void readGPS()
{
    while (Serial3.available())
    {
        gps.encode(Serial3.read());
    }

    if (gps.location.isUpdated())
    {
        latitude = gps.location.lat();
        longitude = gps.location.lng();
    }
}

//-------------------------------------------------------------

void printTelemetry()
{
    Serial2.println("----------------------------");

    Serial2.print("AX : ");
    Serial2.println(ax);

    Serial2.print("AY : ");
    Serial2.println(ay);

    Serial2.print("AZ : ");
    Serial2.println(az);

    Serial2.print("Tilt : ");
    Serial2.println(tilt,1);

    if(gps.location.isValid())
    {
        Serial2.print("Latitude : ");
        Serial2.println(latitude,6);

        Serial2.print("Longitude: ");
        Serial2.println(longitude,6);

        Serial2.print("Satellites: ");
        Serial2.println(gps.satellites.value());
    }
    else
    {
        Serial2.println("Waiting for GPS Fix...");
    }
}

//-------------------------------------------------------------

void setup()
{
    Serial2.begin(115200);
    Serial3.begin(9600);

    Wire.begin();

    initMPU();
}

//-------------------------------------------------------------

void loop()
{
    readMPU();

    readGPS();

    printTelemetry();

    delay(500);
}