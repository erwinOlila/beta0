#include <Wire.h>
#include <RTClib.h>
#include <HardwareSerial.h>

#define PUMP_TIMER 1200000 // 20 minutes

const int TEMP_SCHED[] = {1, 5, 9, 13, 17, 21}; // automatically turn on the pump every 4 hours
const int SCHED_SIZE = sizeof(TEMP_SCHED) / sizeof(TEMP_SCHED[0]); // get the size of the TEMP_SCHED array

int curr = 0; // schedule locator of the TEMP_SCHED array
int temp_pump_timer = 0;
int pump_sensor = 15; // GPIO that monitors the state (ON/OFF) of the pump

HardwareSerial ArduinoSerial(1); // choose UART2
RTC_DS3231 rtc;

int get_time_index();
int get_hour();

void setup() {
  Serial.begin(9600);
  ArduinoSerial.begin(115200, SERIAL_8N1, 16, 17); //RX: 16, TX: 17
  pinMode(15, INPUT);

  if(!rtc.begin()) {
    Serial.println("Cannot find RTC");
    while(1);
  }
  // rtc.adjust(DateTime(__DATE__, __TIME__)); // copy the time from pc to the RTC
  curr = get_time_index();
  Serial.println(curr);
}

void loop() {
  /*
    If the current time, in hours, is greater than the schedule: TEMP_SCHED[curr], the locator
    curr is incremented by 1. ESP32 then signals the arduino to turn on by sending a 'TMP1'.
    Then, the timer temp_pump_timer is started.
    For example, if the current time is 10:00 AM while the curr is 2 (9:00 schedule) the pump must
    turn ON. The increment of the curr and the start of the timer are only triggered during the first 
    start up of the motor in every schedule.

    If the current time is below TEMP_SCHED[curr], status of the motor is checked. If the motor is ON, 
    the time of operation is checked if it has been operating beyond the set duration of PUMP_TIMER. If
    yes, the ESP32 signals the Arduino to turn off by sending a 'TMP0' message
    For example, if the schedule is 13:00 (that is, curr = 3) while the current time is 10:00, and the 
    motor has been running since 9:00 AM, the ESP32 must signal the arduino to tuen off the motor.
  */
  if (get_hour() > TEMP_SCHED[curr]) {
    curr += 1;
    ArduinoSerial.print("TMP1");
    temp_pump_timer = millis();
  } else {
    if (digitalRead(pump_sensor) == HIGH) {
      if ((millis() - temp_pump_timer) > PUMP_TIMER) {
        ArduinoSerial.print("TMP0");
      }
    }
  }
  delay(1000);
}

/*
  Gets the schedule to turn ON the pump at the start up of the device. If the current h is in between
  i and i+1, the locator 's' is set as i. For example, if the current h is 11,
  the s is set as 2 (corresponds to 9:00 AM schedule)
*/
int get_time_index() {
  int i = 0;
  int n = 0;
  int s = 0;

  int h = get_hour();
  
  for (i = 0; i < SCHED_SIZE - 1; i++) {
    if (i == SCHED_SIZE - 1) {
      n = 0;
    }else {
      n = i + 1;
    }
    if (h >= TEMP_SCHED[i] && h < TEMP_SCHED[n]) {
      s = i;
      return s;
    }
  }
}

/*
  Gets the current time in hours
*/
int get_hour() {
  DateTime now = rtc.now();
  int H  = 0;
  H = now.hour();
  return H;
}
