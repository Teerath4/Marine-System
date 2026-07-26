#include <Wire.h>
#include <TinyGPS++.h>

#define MPU_ADDR 0x68//address of mpu
#define LORA_AUX PB14//for lora verfication

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

//-------------------------------------------------------------

bool checkMPU()
{
    Wire.beginTransmission(MPU_ADDR);

    if (Wire.endTransmission() == 0)
    {
        Serial2.println("[PASS] MPU6050 Detected");
        return true;
    }

    Serial2.println("[FAIL] MPU6050 Not Detected");
    return false;
}

//-------------------------------------------------------------

bool checkGPS()
{
    Serial2.print("Checking GPS");

    unsigned long start = millis();

    while (millis() - start < 5000)
    {
        while (Serial3.available())
        {
            gps.encode(Serial3.read());

            if (gps.charsProcessed() > 10)
            {
                Serial2.println("\n[PASS] GPS Communication OK");

                if (gps.location.isValid())
                {
                    Serial2.println("[PASS] GPS Fix Available");
                }
                else
                {
                    Serial2.println("[INFO] GPS Connected (Waiting for Satellite Fix)");
                }

                return true;
            }
        }

        Serial2.print(".");
        delay(250);
    }

    Serial2.println("\n[FAIL] No GPS Data Received");
    return false;
}

//-------------------------------------------------------------

bool checkLoRa()
{
    Serial2.print("Checking LoRa");

    unsigned long start = millis();

    while (millis() - start < 3000)
    {
        if (digitalRead(LORA_AUX) == HIGH)
        {
            Serial2.println("\n[PASS] LoRa Module Ready");
            return true;
        }

        Serial2.print(".");
        delay(250);
    }

    Serial2.println("\n[FAIL] LoRa Module Not Ready");
    return false;
}

void setup()
{
    Serial1.begin(9600);       // LoRa
    Serial2.begin(115200);     // Debug
    Serial3.begin(9600);       // GPS

    pinMode(LORA_AUX, INPUT);

    Wire.begin();

    Serial2.println();
    Serial2.println("========================================");
    Serial2.println(" Marine Safety System Boot");
    Serial2.println("========================================");

    initMPU();

    bool mpuOK  = checkMPU();
    bool gpsOK  = checkGPS();
    bool loraOK = checkLoRa();

    Serial2.println();
    Serial2.println("========== STARTUP REPORT ==========");

    Serial2.print("MPU6050 : ");
    Serial2.println(mpuOK ? "PASS" : "FAIL");

    Serial2.print("GPS     : ");
    Serial2.println(gpsOK ? "PASS" : "FAIL");

    Serial2.print("LoRa    : ");
    Serial2.println(loraOK ? "PASS" : "FAIL");

    if (mpuOK && gpsOK && loraOK)
    {
        Serial2.println();
        Serial2.println("SYSTEM READY");
    }
    else
    {
        Serial2.println();
        Serial2.println("SYSTEM HAS ERRORS");
    }

    Serial2.println("====================================");
    Serial2.println();
}

//-------------------------------------------------------------

void loop()
{
    readMPU();

    readGPS();

    printTelemetry();

    delay(500);
}
