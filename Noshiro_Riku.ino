#include <SPI.h>
#include <Wire.h>
#include <Servo.h>
#include <SD.h>
#include <SoftwareSerial.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_BMP085_U.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_10DOF.h>

/*assgin a unique ID to these sensors at the same time*/
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_BMP085_Unified       bmp = Adafruit_BMP085_Unified(18001);
Adafruit_L3GD20_Unified       gyro = Adafruit_L3GD20_Unified(20);

/*servo */
#define CLOSE 95
#define OPEN 130
Servo servo0;

/*GPS*/
SoftwareSerial sGps(2, 3);

double a[10] = { 0,0,0,0,0,0,0,0,0,0 };

void setup() {
    Serial.begin(9600);
    sGps.begin(9600);
    servo0.attach(5);
    servo0.write(CLOSE); //close door
    pinMode(7, OUTPUT);
    File ff = SD.open("Riku.txt", FILE_WRITE);
    ff.print("accel_x,accel_y,accel_z,pressure,temp");
    ff.close();
    accel.begin();
    bmp.begin();
    gyro.begin();
    sensor_t sensor;
    accel.getSensor(&sensor);
    gyro.getSensor(&sensor);
    bmp.getSensor(&sensor);
    Serial.println(F("0: ServoClose 1:ServoOpen 2:Sensortest 3:SDtest 4:flightmode"));
}

void loop() {
    if (Serial.available() > 0) {
        /*mode select*/
        int mode = Serial.read();
        if (mode == '0') {
            delay(500);
            servo0.write(CLOSE);
            delay(500);
        }
        else if (mode == '1') {
            delay(500);
            servo0.write(OPEN);
            delay(500);
        }
        else if (mode == '2') {
            Sensortest();
        }
        else if (mode == '3') {
            SDtest();
        }
        else if (mode == '4') {
            flightmode();
        }
        Serial.println(F("0: ServoClose 1:ServoOpen 2:Sensortest 3:SDtest 4:flightmode"));
    }
}

/*check the move of sensors*/
void Sensortest() {
    int i;
    if (!accel.begin())
    {
        Serial.print(F("No LSM303 detected!"));
    }
    if (!bmp.begin())
    {
        Serial.print(F("No BMP085 detected!"));
    }
    if (!gyro.begin())
    {
        Serial.print(F("No L3GD20 detected!"));
    }
    for (i = 0;i < 10;i++) {
        Serialprint();
    }
    delay(500);
}

/*check the move of SDcard*/
void SDtest() {
    Serial.print(F("Initializing SD card..."));
    /*SDsocket check*/
    if (!SD.begin(10)) {
        Serial.println(F("initialization failed!"));
        return;
    }
    Serial.println(F("initialization done."));
    File f = SD.open("SDtest.txt", FILE_WRITE);
    /*if file opened*/
    if (f) {
        f.println("test writing OK!");
        f.close();
    }
    else {
        Serial.println(F("error opening SDtest.txt"));
    }
    delay(500);
}

/*filght mode*/
void flightmode() {
    Serial.println(F("flight mode start."));
    sensors_event_t event;
    int ignition;
    int timer = 0;
    int toptime;
    int falltime = 0;

    while (1) {
        accel.getEvent(&event);
        if (IgniteCheck(event)) {
            ignition = millis();
            break;
        }
    }
    while (1) {
        SDprint();
        timer = millis() - ignition;
        bmp.getEvent(&event);
        if (Topcheck(event, timer) || Timer(timer)) {
            toptime = millis();
            break;
        }
    }
    while (1) {
        SDprint();
        timer = millis() - ignition;
        falltime = millis() - toptime;
        if (falltime < 5000) {
            digitalWrite(7, HIGH);
        }
        else { digitalWrite(7, LOW); }
        if (falltime > 500) { servo0.write(OPEN); }
        if (timer > 40000) { break; }
    }
    while (1) {
        SDprint();
        Gpsprint();
    }
}

/*write data to Serial monitor*/
void Serialprint() {
    sensors_event_t event;

    accel.getEvent(&event);
    Serial.print(event.acceleration.y); Serial.print(", ");
    Serial.print(event.acceleration.z); Serial.print(", ");
    Serial.print(event.acceleration.x); Serial.println(" m/s^2 ");

    bmp.getEvent(&event);
    Serial.print(event.pressure);
    Serial.print(" hPa, ");

    float temperature;
    bmp.getTemperature(&temperature);
    Serial.print(temperature);
    Serial.println(" C ");

    gyro.getEvent(&event);
    Serial.print(event.gyro.y); Serial.print(", ");
    Serial.print(event.gyro.z); Serial.print(", ");
    Serial.print(event.gyro.x); Serial.println(" rad/s ");
}

/*write data to SDcard*/
void SDprint() {
    File ff = SD.open("Riku.txt", FILE_WRITE);
    sensors_event_t event;
    ff.print(millis());
    ff.print(", ");

    accel.getEvent(&event);
    ff.print(event.acceleration.y); ff.print(", ");
    ff.print(event.acceleration.z); ff.print(", ");
    ff.print(event.acceleration.x); ff.print(" m/s^2, ");

    bmp.getEvent(&event);
    ff.print(event.pressure); ff.print(" hPa, ");
    float temperature;
    bmp.getTemperature(&temperature);
    ff.print(temperature); ff.print(" C, ");

    gyro.getEvent(&event);
    ff.print(event.gyro.y); ff.print(", ");
    ff.print(event.gyro.z); ff.print(", ");
    ff.print(event.gyro.x); ff.println(" rad/s ");

    ff.close();
}

/*if accel_x>3G ignition*/
bool IgniteCheck(sensors_event_t a) {
    accel.getEvent(&a);
    if (abs(a.acceleration.x) > 29) {
        return true;
    }
    else { return false; }
}

/*move system by barometer change*/
bool Topcheck(sensors_event_t baro, int t) {
    int i = 0;
    int j = 0;

    bmp.getEvent(&baro);

    /*overwrite pressure datas*/
    for (i = 0;i < 9;i++) {
        a[i] = a[i + 1];
    }
    a[9] = baro.pressure;

    /*if baro increases seven times*/
    for (i = 0;i < 9;i++) {
        if (a[i + 1] > a[i]) {
            j++;
            if (j >= 7 && t > 8000) {
                EventCheck(1);
                return true;
            }
            else { return false; }
        }
    }
}

/*move system by timer*/
bool Timer(int t) {
    /*12sec after igniton*/
    if (t > 12000) {
        EventCheck(2);
        return true;
    }
    else { return false; }
}

void EventCheck(int n) {
    File ff = SD.open("Riku.txt", FILE_WRITE);
    ff.print("Event:");
    ff.println(n); 
    ff.close();
}

/*write GPS data to Serial monitor*/
void Gpsprint() {
    char buf[256];
    char c;
    int count = 0;
    float gpsLast_number = 0, gpsLong_number = 0;
    int latitude_i, longitude_i;
    float latitude_f, longitude_f;
    char *gpsTime, *gpsLast, *gpsLong;

    do {
        if (sGps.available()) {
            buf[count] = sGps.read();
            count++;
        }
        if (count > 250) break;
    } while (buf[count - 1] != 0x0A);
    buf[count] = '\0';

    if (0 == strncmp("$GPGGA", buf, 6)) {
        strtok(buf, ",");
        gpsTime = strtok(NULL, ",");
        gpsLast = strtok(NULL, ",");
        strtok(NULL, ",");
        gpsLong = strtok(NULL, ",");
        gpsLast_number = atof(gpsLast) / 100;
        latitude_i = atoi(gpsLast) / 100;
        latitude_f = gpsLast_number - latitude_i;
        latitude_f /= 60;
        latitude_f *= 100;
        gpsLast_number = latitude_i + latitude_f;
        gpsLong_number = atof(gpsLong) / 100;
        longitude_i = atoi(gpsLong) / 100;
        longitude_f = gpsLong_number - longitude_i;
        longitude_f /= 60;
        longitude_f *= 100;
        gpsLong_number = longitude_i + longitude_f;

        Serial.print(F("Latitude = "));
        Serial.print(gpsLast_number);
        Serial.print(" : ");
        Serial.print(F("Longitude = "));
        Serial.println(gpsLong_number);
    }
}
