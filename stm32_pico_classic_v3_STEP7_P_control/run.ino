// Copyright 2026 RT Corporation
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
#include "sensor.h"

RUN g_run;

RUN::RUN() {
  speed = 0.0;
  accel = 0.0;
  con_wall.kp = CON_WALL_KP;
  motor_move=false;
}

//割り込み
void controlInterrupt(void) {
  g_run.interrupt();
}

void stepRInterrupt(void){
  g_run.interruptMotR();
}

void stepLInterrupt(void){
  g_run.interruptMotL();
}

void RUN::interruptMotR(void){
  if(g_run.motor_move){
    Timer2->pause();
    Timer2->setOverflow(g_run.step_hz_r, HERTZ_FORMAT);
    Timer2->resume();     
    
    digitalWrite(PWM_R,HIGH);
    for(int i=0;i<100;i++){
      asm("nop \n");
    }
    digitalWrite(PWM_R,LOW);
    g_run.step_r++;
  }
}

void RUN::interruptMotL(void){
  if(g_run.motor_move){
    Timer3->pause();
    Timer3->setOverflow(g_run.step_hz_l, HERTZ_FORMAT);
    Timer3->resume();     
    
    digitalWrite(PWM_L,HIGH);
    for(int i=0;i<100;i++){
      asm("nop \n");
    }
    digitalWrite(PWM_L,LOW);
    g_run.step_l++;
  }
}


void RUN::interrupt(void) {  //割り込み内からコール

  g_run.speed += g_run.accel;

  if (g_run.speed > max_speed) {
    g_run.speed = max_speed;
  }
  if (g_run.speed < min_speed) {
    g_run.speed = min_speed;
  }

  if ((g_sensor.sen_r.is_control == true) && (g_sensor.sen_l.is_control == true)) {
    con_wall.error = g_sensor.sen_r.error - g_sensor.sen_l.error;
  } else {
    con_wall.error = 2.0 * (g_sensor.sen_r.error - g_sensor.sen_l.error);
  }

  con_wall.control = 0.001 * g_run.speed * con_wall.kp * con_wall.error;

  g_run.speed_target_r = g_run.speed + con_wall.control;
  g_run.speed_target_l = g_run.speed - con_wall.control;

  if (g_run.speed_target_r < min_speed) {
    g_run.speed_target_r = min_speed;
  }

  if (g_run.speed_target_l < min_speed) {
    g_run.speed_target_l = min_speed;
  }
}


void RUN::dirSet(t_CW_CCW dir_left, t_CW_CCW dir_right) {
  if(dir_right==MOT_FORWARD){
    digitalWrite(CW_R,LOW);
  }else{
    digitalWrite(CW_R,HIGH);
  }
  if(dir_right==MOT_FORWARD){
    digitalWrite(CW_L,LOW);
  }else{
    digitalWrite(CW_L,HIGH);
  }
}

void RUN::counterClear(void) {
  g_run.step_r=g_run.step_l=0;
}

void RUN::speedSet(float l_speed, float r_speed) {
  g_run.step_hz_r = r_speed/PULSE;
  g_run.step_hz_l = l_speed/PULSE;
}

void RUN::stepGet(void) {
  g_run.step_lr = g_run.step_r+g_run.step_l;
  g_run.step_lr_len = (int)((float)g_run.step_lr / 2.0 * PULSE);
}

void RUN::stop(void) {
  g_run.motor_move=0;
}

void RUN::accelerate(int len, int finish_speed) {
  int obj_step;

  g_run.accel = 1.5;
  g_run.speed = g_run.min_speed = MIN_SPEED;
  g_run.max_speed = finish_speed;
  counterClear();
  speedSet(MIN_SPEED, MIN_SPEED);
  dirSet(MOT_FORWARD, MOT_FORWARD);
  obj_step = (int)((float)len * 2.0 / PULSE);
  g_run.motor_move=1;
  
  while (1) {
    stepGet();
    speedSet(g_run.speed_target_l, g_run.speed_target_r);
    if (g_run.step_lr > obj_step) {
      break;
    }
  }
}

void RUN::oneStep(int len, int init_speed) {
  int obj_step;

  g_run.accel = 0.0;
  g_run.max_speed = init_speed;
  g_run.speed = g_run.min_speed = init_speed;
  counterClear();
  speedSet(init_speed, init_speed);
  dirSet(MOT_FORWARD, MOT_FORWARD);
  obj_step = (int)((float)len * 2.0 / PULSE);

  while (1) {
    stepGet();
    speedSet(g_run.speed_target_l, g_run.speed_target_r);
    if (g_run.step_lr > obj_step) {
      break;
    }
  }
}

void RUN::decelerate(int len, int init_speed) {
  int obj_step;

  g_run.accel = 1.5;
  g_run.max_speed = init_speed;
  g_run.speed = g_run.min_speed = init_speed;
  counterClear();
  speedSet(init_speed, init_speed);
  dirSet(MOT_FORWARD, MOT_FORWARD);
  obj_step = (int)((float)len * 2.0 / PULSE);

  while (1) {
    stepGet();
    speedSet(g_run.speed_target_l, g_run.speed_target_r);
    if ((int)(len - g_run.step_lr_len) < (int)(((g_run.speed * g_run.speed) - (MIN_SPEED * MIN_SPEED)) / (2.0 * 1000.0 * g_run.accel))) {
      break;
    }
  }

  g_run.accel = -1.5;
  g_run.min_speed = MIN_SPEED;

  while (1) {
    stepGet();
    speedSet(g_run.speed_target_l, g_run.speed_target_r);
    if (g_run.step_lr > obj_step) {
      break;
    }
  }

  stop();
}


void RUN::rotate(t_local_direction dir, int times) {
  int obj_step;

  g_run.accel = 1.5;
  g_run.max_speed = 350.0;
  g_run.speed = g_run.min_speed = MIN_SPEED;
  obj_step = (int)(TREAD_WIDTH * PI / 4.0 * (float)times * 2.0 / PULSE);

  switch (dir) {
    case right:
      dirSet(MOT_FORWARD, MOT_BACK);
      break;
    case left:
      dirSet(MOT_BACK, MOT_FORWARD);
      break;
    default:
      dirSet(MOT_FORWARD, MOT_FORWARD);
      break;
  }

  while (1) {
    stepGet();
    speedSet(g_run.speed, g_run.speed);
    if ((int)((obj_step / 2.0 * PULSE) - g_run.step_lr_len) < (int)(((g_run.speed * g_run.speed) - (MIN_SPEED * MIN_SPEED)) / (2.0 * 1000.0 * g_run.accel))) {
      break;
    }
  }

  g_run.accel = -1.5;
  g_run.min_speed = MIN_SPEED;

  while (1) {
    stepGet();
    speedSet(g_run.speed, g_run.speed);
    if (g_run.step_lr > obj_step) {
      break;
    }
  }

  stop();
}
