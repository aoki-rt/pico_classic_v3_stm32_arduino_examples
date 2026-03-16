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


#include "run.h"

HardwareTimer *Timer6 = new HardwareTimer(TIM6);
HardwareTimer *Timer2 = new HardwareTimer(TIM2);//PWM_R PB10
HardwareTimer *Timer3 = new HardwareTimer(TIM3);//PWM_L PA7

#define LED0 PC11
#define LED1 PC10
#define LED2 PB15
#define LED3 PB14

#define SW_L PC15
#define SW_C PF0
#define SW_R PF1

#define MOTOR_EN PC14
#define CW_R PB1
#define CW_L PC5
#define PWM_R PB10
#define PWM_L PA7

#define MIN_HZ 80
#define TIRE_DIAMETER (48.00)
#define PULSE (TIRE_DIAMETER * PI / 400.0)
#define MIN_SPEED (MIN_HZ*PULSE)

void setup() {
  // put your setup code here, to run once:
  pinMode(LED0, OUTPUT);
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);

  pinMode(SW_L, INPUT_PULLUP);
  pinMode(SW_C, INPUT_PULLUP);
  pinMode(SW_R, INPUT_PULLUP);

  //motor disable
  pinMode(MOTOR_EN, OUTPUT);
  pinMode(CW_R, OUTPUT);
  pinMode(CW_L, OUTPUT);
  pinMode(PWM_R, OUTPUT);
  pinMode(PWM_L, OUTPUT);

  digitalWrite(MOTOR_EN, LOW);
  digitalWrite(CW_R, LOW);
  digitalWrite(CW_L, LOW);
  digitalWrite(PWM_R, LOW);
  digitalWrite(PWM_L, LOW);

  Timer6->pause();
  Timer6->setOverflow(1000, MICROSEC_FORMAT);
  Timer6->attachInterrupt(controlInterrupt);
  Timer6->refresh();
  Timer6->resume();  //Timter Start

  Timer2->pause();
  Timer2->setOverflow(MIN_HZ, HERTZ_FORMAT);
  Timer2->attachInterrupt(stepRInterrupt);
  Timer2->refresh();
  Timer2->resume();  //Timter Start

  Timer3->pause();
  Timer3->setOverflow(MIN_HZ, HERTZ_FORMAT);
  Timer3->attachInterrupt(stepLInterrupt);
  Timer3->refresh();
  Timer3->resume();  //Timter Start



}

void loop() {
  // put your main code here, to run repeatedly:
  while (digitalRead(SW_L) & digitalRead(SW_C) & digitalRead(SW_R)) {
    continue;
  }
  digitalWrite(MOTOR_EN, HIGH);
  delay(1000);
  digitalWrite(LED0, HIGH);
  g_run.accelerate(90, 350);
  digitalWrite(LED1, HIGH);
  digitalWrite(LED2, HIGH);
  g_run.oneStep(180, 350);
  digitalWrite(LED3, HIGH);
  g_run.decelerate(90, 350);
  digitalWrite(LED0, LOW);
  digitalWrite(LED1, LOW);
  digitalWrite(LED2, LOW);
  digitalWrite(LED3, LOW);
  delay(1000);
  digitalWrite(MOTOR_EN, LOW);
}
