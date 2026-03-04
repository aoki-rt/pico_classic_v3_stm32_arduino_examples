// Copyright 2025 RT Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

HardwareTimer *Timer7 = new HardwareTimer(TIM7);

#define SLED_FR PC8
#define SLED_FL PC9
#define SLED_R PA8
#define SLED_L PA15

#define AD_RIGHT_S PA0
#define AD_LEFT_F PA1
#define AD_RIGHT_F PA2
#define AD_LEFT_S PC3
#define AD_VDD PA3

volatile short g_sensor_value_fr;
volatile short g_sensor_value_fl;
volatile short g_sensor_value_r;
volatile short g_sensor_value_l;
volatile short g_battery_value;

void sensorInterrupt(void) {
  static char cnt = 0;
  switch (cnt) {
    case 0:
      digitalWrite(SLED_FR, HIGH);  //LED点灯
      g_sensor_value_fr = RTanalogRead(AD_RIGHT_F);
      digitalWrite(SLED_FR, LOW);  //LED消灯
      break;
    case 1:
      digitalWrite(SLED_FL, HIGH);  //LED点灯
      g_sensor_value_fl = RTanalogRead(AD_LEFT_F);
      digitalWrite(SLED_FL, LOW);  //LED消灯
      break;
    case 2:
      digitalWrite(SLED_R, HIGH);  //LED点灯
      g_sensor_value_r = RTanalogRead(AD_RIGHT_S);
      digitalWrite(SLED_R, LOW);  //LED消灯
      break;
    case 3:
      digitalWrite(SLED_L, HIGH);  //LED点灯
      g_sensor_value_l = RTanalogRead(AD_LEFT_S);
      digitalWrite(SLED_L, LOW);  //LED消灯
      g_battery_value = (float)RTanalogRead(AD_VDD) / 4095.0 * 3300.0 / 10.0 * (10.0 + 51.0);
      break;
    default:
      Serial.printf("error¥n¥r");
      break;
  }
  cnt++;
  if (cnt >= 4) cnt = 0;
}

void setup() {
  // put your setup code here, to run once:
  //Sensor 発光off
  pinMode(SLED_FR, OUTPUT);
  pinMode(SLED_FL, OUTPUT);
  pinMode(SLED_R, OUTPUT);
  pinMode(SLED_L, OUTPUT);
  digitalWrite(SLED_FR, LOW);
  digitalWrite(SLED_FL, LOW);
  digitalWrite(SLED_R, LOW);
  digitalWrite(SLED_L, LOW);

  Serial.begin(115200);

  Timer7->pause();
  Timer7->setOverflow(250, MICROSEC_FORMAT);
  Timer7->attachInterrupt(sensorInterrupt);
  Timer7->refresh();
  Timer7->resume();  //Timter Start
}

void loop() {
  // put your main code here, to run repeatedly:

  Serial.printf("r_sen  is %d\n\r", g_sensor_value_r);
  Serial.printf("fr_sen is %d\n\r", g_sensor_value_fr);
  Serial.printf("fl_sen is %d\n\r", g_sensor_value_fl);
  Serial.printf("l_sen  is %d\n\r", g_sensor_value_l);
  Serial.printf("VDD    is %d\n\r", g_battery_value);
  delay(100);
}
