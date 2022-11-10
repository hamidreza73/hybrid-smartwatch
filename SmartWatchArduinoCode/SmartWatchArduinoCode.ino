#include <Adafruit_BMP085.h>
#include <Adafruit_Sensor.h>
#include <ADXL345.h>
#include <SPI.h>
#include <Wire.h>
#include <math.h>

#include "oled.h"
#include "ble.h"
#include "battery.h"
#include "l3g_custom.h"
#include "calculations.h"

#define SDA_1 21
#define SCL_1 22
#define SDA_2 13
#define SCL_2 27

#define circle_key       32
#define circle_key_long  132
#define square_key       33
#define square_key_long  133
#define left_key         25
#define left_key_long    125
#define right_key        26
#define right_key_long   126
#define no_key           0
#define long_press       100


unsigned int prev_millis = 0;
int last_pressed_key = no_key;
int last_pressed_key_private = no_key;
bool pressed_right_now = true;
bool last_key_is_released = true;

#define everyday_mode  0
#define mountain_mode  1
#define bicycle_mode   2
#define exercise_mode  3
int watch_mode;
int sub_mode;

int dim_time = 15000; //millis

int speed;
bool displays_on = true;
int displays_on_millis;

int steps_count;
int calories_burnt;
int run_dist;

int bicycle_dist; //km

float heading;
#define HMC5883_address 0x1E //0011110b, I2C 7bit address of HMC5883
static float hmc5883_Gauss_LSB_XY = 1100.0F; // Varies with gain
static float hmc5883_Gauss_LSB_Z = 980.0F;   // Varies with gain
#define SENSORS_GAUSS_TO_MICROTESLA 100

Adafruit_BMP085 bmp;

ADXL345 accel(ADXL345_ALT, &I2Cone);

L3G gyro;
#define wakeup_threshold 30000

int stopwatch_start_millis;
bool stopwatch_is_started = false;
int elapsed_millis;
int last_elapsed_millis;
String elapsed_deci_str;
int elapsed_second;
String elapsed_second_str ;
int elapsed_minute ;
String elapsed_minute_str;
int elapsed_hour;
String elapsed_hour_str;


String date_month = "FEB"; //FEB,JAN,...
int date_day = 7;


void setup()
{

  pinMode(circle_key, INPUT_PULLUP);
  pinMode(square_key, INPUT_PULLUP);
  pinMode(left_key, INPUT_PULLUP);
  pinMode(right_key, INPUT_PULLUP);


  I2Cone.begin(SDA_1, SCL_1, 100000);
  I2Ctwo.begin(SDA_2, SCL_2, 100000);
  //Wire.begin(27, 13);

  Serial.begin(115200);

  displaysSetup();

  //mode_select();

  delay(50);

  ////////////gyro////////////////

  if (!gyro.init())
  {
    delay(100);
    Serial.println("Failed to autodetect gyro type!");
    while (1);
  }
  delay(100);

  gyro.enableDefault();


  //////////////accel////////////////
  byte deviceID = accel.readDeviceID();
  if (deviceID != 0) {
    Serial.print("0x");
    Serial.print(deviceID, HEX);
    Serial.println("");
  } else {
    Serial.println("read device id: failed");
    while (1) {
      delay(100);
    }
  }

  // Data Rate
  // - ADXL345_RATE_3200HZ: 3200 Hz
  // - ADXL345_RATE_1600HZ: 1600 Hz
  // - ADXL345_RATE_800HZ:  800 Hz
  // - ADXL345_RATE_400HZ:  400 Hz
  // - ADXL345_RATE_200HZ:  200 Hz
  // - ADXL345_RATE_100HZ:  100 Hz
  // - ADXL345_RATE_50HZ:   50 Hz
  // - ADXL345_RATE_25HZ:   25 Hz
  // - ...
  if (!accel.writeRate(ADXL345_RATE_200HZ)) {
    Serial.println("write rate: failed");
    while (1) {
      delay(100);
    }
  }

  // Data Range
  // - ADXL345_RANGE_2G: +-2 g
  // - ADXL345_RANGE_4G: +-4 g
  // - ADXL345_RANGE_8G: +-8 g
  // - ADXL345_RANGE_16G: +-16 g
  if (!accel.writeRange(ADXL345_RANGE_16G)) {
    Serial.println("write range: failed");
    while (1) {
      delay(100);
    }
  }

  if (!accel.start()) {
    Serial.println("start: failed");
    while (1) {
      delay(100);
    }
  }

  ////////////////////pressure////////////
  if (!bmp.begin(1, &I2Cone)) {//1 for standard mode
    Serial.println("Could not find a valid BMP085 sensor, check wiring!");
    while (1) {}
  }

  ////////////////////magneto////////////////
  //Put the HMC5883 IC into the correct operating mode
  I2Cone.beginTransmission(HMC5883_address); //open communication with HMC5883
  I2Cone.write(0x02); //select mode register
  I2Cone.write(0x00); //continuous measurement mode
  I2Cone.endTransmission();
}



///////////////////////loop ///////////////////////////////////////////////////////////////
void loop() {
  ////////////////////gyro////////////////////
  gyro.read();
  Serial.print("G ");
  Serial.print("X: ");
  Serial.print((int)gyro.g.x);
  Serial.print(" Y: ");
  Serial.print((int)gyro.g.y);
  Serial.print(" Z: ");
  Serial.print((int)gyro.g.z);

  //delay(100);

  ////////////////////acelero/////////////////
  accel.update();
  Serial.print("   A ");
  Serial.print("X: ");
  Serial.print(accel.getX(), 2);
  Serial.print(" Y: ");
  Serial.print(accel.getY(), 2);
  Serial.print(" Z: ");
  Serial.println(accel.getZ(), 2);


  if ((gyro.g.x > wakeup_threshold) || (gyro.g.y > wakeup_threshold) || (gyro.g.z > wakeup_threshold) || (gyro.g.x < -wakeup_threshold) || (gyro.g.y < -wakeup_threshold) || (gyro.g.z < -wakeup_threshold)) {
    displays_on = true;
    displays_on_millis = millis();
  }
  if ((millis() - displays_on_millis) > dim_time) {
    displays_on = false;
  }

  //////////////////check for key press//////////////////
  if (pressed_key() != no_key) {
    if (displays_on == false) {
      displays_on = true;
      displays_on_millis = millis();

    } else {
      displays_on_millis = millis();

      if (last_pressed_key == square_key) {
        mode_select();

      } else if (last_pressed_key == square_key_long) {
        settings();

      }  else if (last_pressed_key == left_key) {

        if (sub_mode == 0)
          sub_mode = 4;
        else
          sub_mode = sub_mode - 1;

      } else if (last_pressed_key == right_key) {

        if (sub_mode == 4)
          sub_mode = 0;
        else
          sub_mode = sub_mode + 1;
      }
    }
  }

  //////////////////////display////////////////////////////
  if (  displays_on == true) {
    if (watch_mode == everyday_mode) {

      if      (sub_mode == 0)
        date();

      else if (sub_mode == 1)
        steps_calories();

      else if (sub_mode == 2)
        weather();

      else if (sub_mode == 3)
        notification();

      else if (sub_mode == 4)
        music();

    } else if (watch_mode == bicycle_mode) {
      bicycle_display_up();

      if      (sub_mode == 0)
        bicycle_navigation();

      else if (sub_mode == 1)
        hidration();

      else if (sub_mode == 2)
        bpm_o2();

      else if (sub_mode == 3)
        weather();

      else if (sub_mode == 4)
        music();

    } else if (watch_mode == mountain_mode) {
      mountain_display_up();

      if      (sub_mode == 0)
        bpm_o2();

      else if (sub_mode == 1)
        steps_calories();

      else if (sub_mode == 2)
        mountain_route();

      else if (sub_mode == 3)
        compass();

      else if (sub_mode == 4)
        weather();

    } else if (watch_mode == exercise_mode) {
      exercise_display_up();

      if      (sub_mode == 0)
        steps_calories();

      else if (sub_mode == 1)
        bpm_o2();

      else if (sub_mode == 2)
        stopwatch();

      else if (sub_mode == 3)
        reps();

      else if (sub_mode == 4)
        music();

    } else {
      mode_select();
    }
  } else {
    displayUp.clearDisplay();
    displayUp.display();
    displayDown.clearDisplay();
    displayDown.display();

  }
  //delay(50);
}
/////////////////////////////// modes//////////////////////////////////////////////////////////////////////

void mountain_display_up() {

  displayUp.clearDisplay();
  display_up_left(battery_percentage, missedcall_count, message_count);

  ////////////temprature//////////////
  int temp = bmp.readTemperature();
  displayUp.drawBitmap(110, 0, temp_icon, 16, 16, 1);
  displayUp.setCursor(67, 12);
  displayUp.print(String(temp) /*+ char(247)*/ + "C");

  ////////////altitude////////////////
  int altitude = bmp.readAltitude();
  displayUp.drawBitmap(110, 15, altitude_icon, 16, 16, 1);
  if (altitude > 1000) {
    displayUp.setCursor(45, 28);
    displayUp.print(altitude + "m");
  }  else if (altitude > 100 ) {
    displayUp.setCursor(56, 28);
    displayUp.print(String(altitude) + "m");
  }  else if ((altitude > 10 ) || (altitude < 0 )) {
    displayUp.setCursor(67, 28);
    displayUp.print(String(altitude) + "m");
  } else {
    displayUp.setCursor(78, 28);
    displayUp.print("0" + String(altitude) + "m");
  }

  ////////////heading/////////////////
  calculate_heading();

  displayUp.drawBitmap(110, 31, compass_icon_16, 16, 16, 1);
  displayUp.setCursor(78, 44);
  if ((heading < 22.5) || (heading > 337.5))
    displayUp.print("N");
  else if (heading < 67.5)
    displayUp.print("NE");
  else if (heading < 112.5)
    displayUp.print("E");
  else if (heading < 157.5)
    displayUp.print("SE");
  else if (heading < 202.5)
    displayUp.print("S");
  else if (heading < 247.5)
    displayUp.print("SW");
  else if (heading < 292.5)
    displayUp.print("W");
  else if (heading < 337.5)
    displayUp.print("NW");

  ////////////speed////////////////////////////////////////
  displayUp.drawBitmap(110, 48, speed_icon, 16, 16, 1);
  displayUp.setCursor(45, 60);
  displayUp.print(String(speed) + "Km/h");


  displayUp.display();

}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void bicycle_display_up() {

  displayUp.clearDisplay();
  display_up_left(battery_percentage, missedcall_count, message_count);

  ////////////temprature///////////////////////////////////
  int temp = bmp.readTemperature();
  displayUp.drawBitmap(110, 0, temp_icon, 16, 16, 1);
  displayUp.setCursor(70, 12);
  displayUp.print(String(temp) /*+ char(247)*/ + "C");

  ////////////speed////////////////////////////////////////
  displayUp.drawBitmap(110, 16, speed_icon, 16, 16, 1);
  displayUp.setCursor(48, 28);
  displayUp.print(String(speed) + "Km/h");

  ////////////dist/////////////////////////////////////////
  displayUp.drawBitmap(110, 32, distance_icon, 16, 16, 1);
  displayUp.setCursor(70, 44);
  displayUp.print(String(bicycle_dist) + "Km");

  //////////calories///////////////////////////////////////
  displayUp.drawBitmap(110, 48, calories_icon_16, 16, 16, 1);
  displayUp.setCursor(59, 60);
  displayUp.print(String(calories_burnt) + "Cal");


  displayUp.display();

}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
void exercise_display_up() {

  displayUp.clearDisplay();
  display_up_left(battery_percentage, missedcall_count, message_count);

  ////////////temprature//////////////
  int temp = bmp.readTemperature();
  displayUp.drawBitmap(110, 0, temp_icon, 16, 16, 1);
  displayUp.setCursor(58, 12);
  displayUp.print(String(temp) /*+ char(247)*/ + "C");

  ////////////speed////////////////////////////////////////
  displayUp.drawBitmap(110, 48, speed_icon, 16, 16, 1);
  displayUp.setCursor(45, 60);
  displayUp.print(String(speed) + "Km/h");

  ////////////dist/////////////////////////////////////////
  displayUp.drawBitmap(110, 32, distance_icon, 16, 16, 1);
  displayUp.setCursor(70, 44);
  displayUp.print(String(run_dist) + "Km");

  //////////calories///////////////////////////////////////
  displayUp.drawBitmap(110, 16, calories_icon_16, 16, 16, 1);
  displayUp.setCursor(61, 28);
  displayUp.print(String(calories_burnt) + "Cal");


  displayUp.display();

}

////////////////////////////////////////////////////////////////////////////////////////////////////////


void notification() { //calls and messages

  dim_time = 15000;

  displayUp.clearDisplay();
  display_up_left(battery_percentage, missedcall_count, message_count, ((watch_mode == everyday_mode) && (sub_mode == 3) ? false : true));

  displayUp.setCursor(92, 14);
  displayUp.print(date_month);

  displayUp.setCursor(103, 30);
  displayUp.print(date_day < 9 ? ("0" + String(date_day)) : String(date_day));


  displayUp.display();

  if (!phone_is_connected) {
    displayDown.clearDisplay();
    //    displayDownCenter("Phone is NOT connected");
    displayDown.setCursor(5, 15);
    displayDown.print(F("Phone NOT"));
    displayDown.setCursor(5, 31);
    displayDown.print(F("connected"));
    displayDown.display();
  } else {
    displayDown.clearDisplay();
    displayDown.drawBitmap(18, 0, missedcall_icon, 40, 40, 1);
    displayDown.setCursor(33, 58);
    displayDown.print(missedcall_count);
    displayDown.drawBitmap(72, 0, message_icon, 40, 40, 1);
    displayDown.setCursor(85, 58);
    displayDown.print(message_count);
    displayDown.display();
  }
  return;
}

void steps_calories() {

  if (watch_mode == everyday_mode) {

    displayUp.clearDisplay();
    display_up_left(battery_percentage, missedcall_count, message_count);

    displayUp.setCursor(92, 14);
    displayUp.print(date_month);

    displayUp.setCursor(104, 30);
    displayUp.print(date_day < 9 ? ("0" + String(date_day)) : String(date_day));


    displayUp.display();

  }

  dim_time = 15000;

  displayDown.clearDisplay();
  displayDown.drawBitmap(18, 0, steps_icon, 40, 40, 1);
  displayDown.setCursor(33, 58);
  displayDown.print(steps_count);
  displayDown.drawBitmap(72, 0, calories_icon, 40, 40, 1);
  displayDown.setCursor(85, 58);
  displayDown.print(calories_burnt);
  displayDown.display();

  return;
}

void date() {
  dim_time = 15000;

  displayUp.clearDisplay();
  display_up_left(battery_percentage, missedcall_count, message_count, true, true);
  displayUp.display();

  displayDown.clearDisplay();
  displayDown.setCursor(44, 26);
  displayDown.setFont(&FreeMonoBold12pt7b);
  date_month.toUpperCase();
  displayDown.print(date_month);
  displayDown.setFont(&FreeMonoBold9pt7b);
  displayDown.setCursor(54, 40);
  displayDown.setFont(&FreeMono9pt7b);
  String day_string = (date_day < 10) ? ("0" + String(date_day)) : (String(date_day));
  displayDown.print(day_string);
  displayDown.display();
  return;
}

void weather() {
  dim_time = 15000;


  if (watch_mode == everyday_mode) {

    displayUp.clearDisplay();
    display_up_left(battery_percentage, missedcall_count, message_count);

    displayUp.setCursor(92, 14);
    displayUp.print(date_month);

    displayUp.setCursor(104, 30);
    displayUp.print(date_day < 9 ? ("0" + String(date_day)) : String(date_day));


    displayUp.display();

  }


  if (!weather_is_valid) {
    displayDown.clearDisplay();
    //    displayDownCenter("Phone is NOT connected");
    displayDown.setCursor(0, 19);
    displayDown.print(F("Weather"));
    displayDown.setCursor(48, 35);
    displayDown.print(F("NOT"));
    displayDown.setCursor(20, 51);
    displayDown.print(F("Aavailable"));
    displayDown.display();
  } else {
    displayDown.clearDisplay();
    displayDown.drawBitmap(18, 0, missedcall_icon, 40, 40, 1);
    displayDown.setCursor(33, 46);
    displayDown.print(missedcall_count);
    displayDown.drawBitmap(72, 0, message_icon, 40, 40, 1);
    displayDown.setCursor(85, 46);
    displayDown.print(message_count);
    displayDown.display();
  }
}

void music() {
  dim_time = 15000;

  if (!phone_is_connected) {
    displayDown.clearDisplay();
    //    displayDownCenter("Phone is NOT connected");
    displayDown.setCursor(5, 15);
    displayDown.print(F("Phone NOT"));
    displayDown.setCursor(5, 31);
    displayDown.print(F("connected"));
    displayDown.display();
  } else {
    displayDown.clearDisplay();
    displayDown.drawBitmap(18, 0, missedcall_icon, 40, 40, 1);
    displayDown.setCursor(33, 58);
    displayDown.print(missedcall_count);
    displayDown.drawBitmap(72, 0, message_icon, 40, 40, 1);
    displayDown.setCursor(85, 58);
    displayDown.print(message_count);
    displayDown.display();
  }
}

void bicycle_navigation() { //turns, ETA, trafic alerts
  dim_time = 40000;

  if (!phone_is_connected) {
    displayDown.clearDisplay();
    //    displayDownCenter("Phone is NOT connected");
    displayDown.setCursor(5, 15);
    displayDown.print(F("Phone NOT"));
    displayDown.setCursor(5, 31);
    displayDown.print(F("connected"));
    displayDown.display();
  } else {
    displayDown.clearDisplay();
    displayDown.drawBitmap(18, 0, missedcall_icon, 40, 40, 1);
    displayDown.setCursor(33, 58);
    displayDown.print(missedcall_count);
    displayDown.drawBitmap(72, 0, message_icon, 40, 40, 1);
    displayDown.setCursor(85, 58);
    displayDown.print(message_count);
    displayDown.display();
  }
}

void hidration() {
  dim_time = 15000;

  if (!phone_is_connected) {
    displayDown.clearDisplay();
    //    displayDownCenter("Phone is NOT connected");
    displayDown.setCursor(5, 15);
    displayDown.print(F("Phone NOT"));
    displayDown.setCursor(5, 31);
    displayDown.print(F("connected"));
    displayDown.display();
  } else {
    displayDown.clearDisplay();
    displayDown.drawBitmap(18, 0, missedcall_icon, 40, 40, 1);
    displayDown.setCursor(33, 58);
    displayDown.print(missedcall_count);
    displayDown.drawBitmap(72, 0, message_icon, 40, 40, 1);
    displayDown.setCursor(85, 58);
    displayDown.print(message_count);
    displayDown.display();
  }
}

void bpm_o2() {
  dim_time = 15000;

  displayDown.clearDisplay();
  displayDown.drawBitmap(18, 0, bpm_icon, 40, 40, 1);
  displayDown.setCursor(33, 58);
  displayDown.print(steps_count);
  displayDown.drawBitmap(72, 0, o2_icon, 40, 40, 1);
  displayDown.setCursor(85, 58);
  displayDown.print(calories_burnt);
  displayDown.display();

  return;
}

void compass() { //distance, attitude difference, ETA
  dim_time = 60000;

  displayDown.clearDisplay();
  if ((heading < 5) || (heading > 355))
    displayDown.drawBitmap(0, 11, compass_0, 40, 40, 1);
  else if (heading > 345)
    displayDown.drawBitmap(0, 11, compass_10, 40, 40, 1);
  else if (heading > 335)
    displayDown.drawBitmap(0, 11, compass_20, 40, 40, 1);
  else if (heading > 325)
    displayDown.drawBitmap(0, 11, compass_30, 40, 40, 1);
  else if (heading > 315)
    displayDown.drawBitmap(0, 11, compass_40, 40, 40, 1);
  else if (heading > 305)
    displayDown.drawBitmap(0, 11, compass_50, 40, 40, 1);
  else if (heading > 295)
    displayDown.drawBitmap(0, 11, compass_60, 40, 40, 1);
  else if (heading > 285)
    displayDown.drawBitmap(0, 11, compass_70, 40, 40, 1);
  else if (heading > 275)
    displayDown.drawBitmap(0, 11, compass_80, 40, 40, 1);
  else if (heading > 265)
    displayDown.drawBitmap(0, 11, compass_90, 40, 40, 1);
  else if (heading > 255)
    displayDown.drawBitmap(0, 11, compass_100, 40, 40, 1);
  else if (heading > 245)
    displayDown.drawBitmap(0, 11, compass_110, 40, 40, 1);
  else if (heading > 235)
    displayDown.drawBitmap(0, 11, compass_120, 40, 40, 1);
  else if (heading > 225)
    displayDown.drawBitmap(0, 11, compass_130, 40, 40, 1);
  else if (heading > 215)
    displayDown.drawBitmap(0, 11, compass_140, 40, 40, 1);
  else if (heading > 205)
    displayDown.drawBitmap(0, 11, compass_150, 40, 40, 1);
  else if (heading > 195)
    displayDown.drawBitmap(0, 11, compass_160, 40, 40, 1);
  else if (heading > 185)
    displayDown.drawBitmap(0, 11, compass_170, 40, 40, 1);
  else if (heading > 175)
    displayDown.drawBitmap(0, 11, compass_180, 40, 40, 1);
  else if (heading > 165)
    displayDown.drawBitmap(0, 11, compass_190, 40, 40, 1);
  else if (heading > 155)
    displayDown.drawBitmap(0, 11, compass_200, 40, 40, 1);
  else if (heading > 145)
    displayDown.drawBitmap(0, 11, compass_210, 40, 40, 1);
  else if (heading > 135)
    displayDown.drawBitmap(0, 11, compass_220, 40, 40, 1);
  else if (heading > 125)
    displayDown.drawBitmap(0, 11, compass_230, 40, 40, 1);
  else if (heading > 115)
    displayDown.drawBitmap(0, 11, compass_240, 40, 40, 1);
  else if (heading > 105)
    displayDown.drawBitmap(0, 11, compass_250, 40, 40, 1);
  else if (heading > 95)
    displayDown.drawBitmap(0, 11, compass_260, 40, 40, 1);
  else if (heading > 85)
    displayDown.drawBitmap(0, 11, compass_270, 40, 40, 1);
  else if (heading > 75)
    displayDown.drawBitmap(0, 11, compass_280, 40, 40, 1);
  else if (heading > 65)
    displayDown.drawBitmap(0, 11, compass_290, 40, 40, 1);
  else if (heading > 55)
    displayDown.drawBitmap(0, 11, compass_300, 40, 40, 1);
  else if (heading > 45)
    displayDown.drawBitmap(0, 11, compass_310, 40, 40, 1);
  else if (heading > 35)
    displayDown.drawBitmap(0, 11, compass_320, 40, 40, 1);
  else if (heading > 25)
    displayDown.drawBitmap(0, 11, compass_330, 40, 40, 1);
  else if (heading > 15)
    displayDown.drawBitmap(0, 11, compass_340, 40, 40, 1);
  else if (heading > 5)
    displayDown.drawBitmap(0, 11, compass_350, 40, 40, 1);
  displayDown.setCursor(48, 38);
  displayDown.setFont(&FreeMonoBold12pt7b);
  displayDown.print(String(heading, 0));
  if ((heading < 22.5) || (heading > 337.5))
    displayDown.print("N");
  else if (heading < 67.5)
    displayDown.print("NE");
  else if (heading < 112.5)
    displayDown.print("E");
  else if (heading < 157.5)
    displayDown.print("SE");
  else if (heading < 202.5)
    displayDown.print("S");
  else if (heading < 247.5)
    displayDown.print("SW");
  else if (heading < 292.5)
    displayDown.print("W");
  else if (heading < 337.5)
    displayDown.print("NW");
  displayDown.setFont(&FreeMono9pt7b);
  displayDown.display();

}

void stopwatch() {
  dim_time = 60000;

  displayDown.clearDisplay();
  displayDown.drawBitmap(0, 11, stopwatch_icon, 40, 40, 1);
  displayDown.setCursor(45, 38);

  if ((last_pressed_key == circle_key) && (!stopwatch_is_started)) {
    //    displayUp.setCursor(24, 4);
    //    displayUp.drawBitmap(100, 11, play_icon_small, 40, 40, 1);
    //    displayUp.display();
    stopwatch_start_millis = millis();
    stopwatch_is_started = true; //toggle

  } else if ((last_pressed_key == circle_key) && (stopwatch_is_started)) {
    //    displayUp.setCursor(24, 4);
    //    displayUp.drawBitmap(100, 11, pause_icon_small, 40, 40, 1);
    //    displayUp.display();
    stopwatch_is_started = false; //toggle
    last_elapsed_millis = elapsed_millis;
  }
  if (last_pressed_key == circle_key_long) {
    stopwatch_is_started = false;
    elapsed_millis = 0;
    last_elapsed_millis = 0;
  }

  if (stopwatch_is_started) {
    elapsed_millis = millis() - stopwatch_start_millis + last_elapsed_millis;
    elapsed_deci_str = String((elapsed_millis % 1000) / 100);
    elapsed_second = (elapsed_millis / 1000) % 60;
    elapsed_second_str = elapsed_second < 10 ? ("0" + String(elapsed_second)) : (String(elapsed_second));
    elapsed_minute = (elapsed_millis / 60000);
    elapsed_minute_str = elapsed_minute < 10 ? ("0" + String(elapsed_minute)) : (String(elapsed_minute));
    elapsed_hour = (elapsed_millis / 3600000);
    elapsed_hour_str = String(elapsed_hour);

    //    if (elapsed_hour > 99) {
    //      displayDown.print(elapsed_hour_str+":"+elapsed_minute_str);
    //    } else
    if (elapsed_hour > 9) {
      displayDown.print(elapsed_hour_str + ":" + elapsed_minute_str);
    } else if (elapsed_hour > 0) {
      displayDown.print(elapsed_hour_str + ":" + elapsed_minute_str + ":" + elapsed_second_str);
    } else if (elapsed_hour == 0) {
      displayDown.print(elapsed_minute_str + ":" + elapsed_second_str + ":" + elapsed_deci_str);
    }
  } else {
    elapsed_deci_str = String((elapsed_millis % 1000) / 100);
    elapsed_second = (elapsed_millis / 1000) % 60;
    elapsed_second_str = elapsed_second < 10 ? ("0" + String(elapsed_second)) : (String(elapsed_second));
    elapsed_minute = (elapsed_millis / 60000);
    elapsed_minute_str = elapsed_minute < 10 ? ("0" + String(elapsed_minute)) : (String(elapsed_minute));
    elapsed_hour = (elapsed_millis / 3600000);
    elapsed_hour_str = String(elapsed_hour);
    if (elapsed_hour > 9) {
      displayDown.print(elapsed_hour_str + ":" + elapsed_minute_str);
    } else if (elapsed_hour > 0) {
      displayDown.print(elapsed_hour_str + ":" + elapsed_minute_str + ":" + elapsed_second_str);
    } else if (elapsed_hour == 0) {
      displayDown.print(elapsed_minute_str + ":" + elapsed_second_str + ":" + elapsed_deci_str);
    }
  }

  displayDown.display();

  return;
}

void mountain_route() {
  dim_time = 60000;

  displayDown.clearDisplay();
  //    displayDownCenter("Phone is NOT connected");
  displayDown.setCursor(24, 27);
  displayDown.print(F("GPS NOT"));
  displayDown.setCursor(12, 43);
  displayDown.print(F("Connected"));
  displayDown.display();
}

void reps() {
  dim_time = 60000;

  displayDown.clearDisplay();
  displayDown.drawBitmap(18, 0, missedcall_icon, 40, 40, 1);
  displayDown.setCursor(33, 46);
  displayDown.print(missedcall_count);
  displayDown.drawBitmap(72, 0, message_icon, 40, 40, 1);
  displayDown.setCursor(85, 46);
  displayDown.print(message_count);
  displayDown.display();

}

////////////////////////////////////////////////////////////////////////////////////////////////////

void mode_select() {
  dim_time = 15000;

  sub_mode = 0;

  prev_millis = millis();
  while ((pressed_key() == no_key) && (millis() - prev_millis <= 10000)) {
    displayUp.clearDisplay();
    displayUp.drawBitmap(0, 0, bicycle_icon, 50, 50, 1);
    displayUp.drawBitmap(78, 0, everyday_icon, 50, 50, 1);
    displayUp.display();


    displayDown.clearDisplay();
    displayDown.drawBitmap(0, 22, mountain_icon, 50, 50, 1);
    displayDown.drawBitmap(82, 20, exercise_icon, 50, 50, 1);
    displayDown.display();

  }

  if (last_pressed_key == circle_key) {
    watch_mode = everyday_mode;
    displayUp.fillCircle(128, 0, 64, WHITE);
    displayUp.display();
    delay(10);
    return;
  }
  else if (last_pressed_key == square_key) {
    watch_mode = bicycle_mode;
    displayUp.fillCircle(0, 0, 64, WHITE);
    displayUp.display();
    delay(10);
    return;
  }
  else if (last_pressed_key == right_key) {
    watch_mode = exercise_mode;
    displayDown.fillCircle(128, 64, 64, WHITE);
    displayDown.display();
    delay(10);
    return;
  }
  else if (last_pressed_key == left_key) {
    watch_mode = mountain_mode;
    displayDown.fillCircle(0, 64, 64, WHITE);
    displayDown.display();
    delay(10);
    return;
  }
}

void settings() {

}


int pressed_key() {

  if (last_key_is_released) {
    if (digitalRead(circle_key) == LOW) {
      last_pressed_key_private = circle_key;
      prev_millis = millis();
      last_key_is_released = false;

    } else if (digitalRead(square_key) == LOW) {
      last_pressed_key_private = square_key;
      prev_millis = millis();
      last_key_is_released = false;


    } else if (digitalRead(left_key) == LOW) {
      last_pressed_key_private = left_key;
      prev_millis = millis();
      last_key_is_released = false;


    } else if (digitalRead(right_key) == LOW) {
      last_pressed_key_private = right_key;
      prev_millis = millis();
      last_key_is_released = false;

    }
  }

  if ((millis() - prev_millis > 1500) && (!last_key_is_released)) {
    last_pressed_key_private = circle_key_long;
  }
  
  if ((digitalRead((last_pressed_key_private > 100) ? (last_pressed_key_private - 100) : last_pressed_key_private) == HIGH)&& (!last_key_is_released))  {
    last_key_is_released = true;
    last_pressed_key = last_pressed_key_private;
    return last_pressed_key;
  }

  last_pressed_key = no_key;
  return no_key;

}

float calculate_heading() {

  int x, y, z; //triple axis data
  int xmin, xmax, ymin, ymax, zmin, zmax;
  xmin = 0; xmax = 0; ymax = 0; ymin = 0; zmin = 0; zmax = 0;

  //Tell the HMC5883 where to begin reading data
  I2Cone.beginTransmission(HMC5883_address);
  I2Cone.write(0x03); //select register 3, X MSB register
  I2Cone.endTransmission(false);

  //Read data from each axis, 2 registers per axis
  I2Cone.requestFrom(HMC5883_address, 6, true);

  // Note high before low (different than accel)
  uint8_t xhi = I2Cone.read();
  uint8_t xlo = I2Cone.read();
  uint8_t zhi = I2Cone.read();
  uint8_t zlo = I2Cone.read();
  uint8_t yhi = I2Cone.read();
  uint8_t ylo = I2Cone.read();

  // Shift values to create properly formed integer (low byte first)
  x = (int16_t)(xlo | ((int16_t)xhi << 8));
  y = (int16_t)(ylo | ((int16_t)yhi << 8));
  z = (int16_t)(zlo | ((int16_t)zhi << 8));

  x = x / hmc5883_Gauss_LSB_XY * SENSORS_GAUSS_TO_MICROTESLA;
  y = y / hmc5883_Gauss_LSB_XY * SENSORS_GAUSS_TO_MICROTESLA;
  z = z / hmc5883_Gauss_LSB_Z * SENSORS_GAUSS_TO_MICROTESLA;

  y = y + 30;
  //int mag_x=event.magnetic.x+30;


  /* Display the results (magnetic vector values are in micro-Tesla (uT)) */
  Serial.print("X: "); Serial.print(x); Serial.print("  ");
  Serial.print("Y: "); Serial.print(y); Serial.print("  ");
  Serial.print("Z: "); Serial.print(z); Serial.print("  "); Serial.println("uT");
  // Hold the module so that Z is pointing 'up' and you can measure the heading with x&y
  // Calculate heading when the magnetometer is level, then correct for signs of axis.
  heading = atan2(y, x);

  // Once you have your heading, you must then add your 'Declination Angle', which is the 'Error' of the magnetic field in your location.
  // Find yours here: http://www.magnetic-declination.com/
  // Mine is: -13* 2' W, which is ~13 Degrees, or (which we need) 0.22 radians
  // If you cannot find your Declination, comment out these two lines, your compass will be slightly off.
  //  float declinationAngle =  0.05236;
  //  heading += declinationAngle;

  heading = -heading + PI; //according to north

  // Correct for when signs are reversed.
  if (heading < 0)
    heading += 2 * PI;

  // Check for wrap due to addition of declination.
  if (heading > 2 * PI)
    heading -= 2 * PI;

  // Convert radians to degrees for readability.
  heading = (heading * 180 / M_PI) - 1;

  //Serial.print("Heading (degrees): "); Serial.println(heading);
  //  delay(500);
  return heading;
}
