#include "Arduino.h"
#include "AX12A.h"     // https://github.com/jumejume1/AX-12A-servo-library

#define DirectionPin  (7u)
#define BaudRate      (1000000ul)
#define LEFT_ID 13
#define RIGHT_ID 1
// 13 - левый, 1 - правый. Смотреть со стороны написанных ID

#define DEFAULT_WORK_SPEED 50
#define DEFAULT_CLOSE_L 550 // положение закрытия левой губки
#define DEFAULT_CLOSE_R 474 // положение закрытия правой губки
#define DEFAULT_OPEN 200    // отклонения положения открытия от положения закрытия (open_left = DEFAULT_CLOSE_L-DEFAULT_OPEN) 

// коды поддерживаемых команд 
#define SEND_GET_POS 10
#define SEND_GET_LOAD 20
#define SEND_GET_VOLT 30 
#define SEND_GET_TEMP 40
#define SEND_GET_COMP_MOVE 45

#define BACK_GET_POS 50
#define BACK_GET_LOAD 60
#define BACK_GET_VOLT 70 
#define BACK_GET_TEMP 80
#define BACK_GET_COMP_MOVE 85

#define START_PACKAGE_FLAG 255

#define RELEASE 100
#define UNRELEASE 101
#define SEND_OPEN 110
#define SEND_CLOSE 111
#define SEND_OPEN_SPEED 120
#define SEND_CLOSE_SPEED 121
#define SEND_ANGLE 130
#define SEND_LEFT_ANGLE 131
#define SEND_RIGHT_ANGLE 132
#define SEND_ANGLE_SPEED 140
#define SEND_LEFT_ANGLE_SPEED 141
#define SEND_RIGHT_ANGLE_SPEED 142

#define MAX_SPEED 1023
#define MIN_SPEED 0

#define MAX_ANGLE 1023
#define MIN_ANGLE 0

#define LOAD_POSITION 50
#define CURRRENT 60
#define VOLTAGE 70
#define ERROR_CHECK_POSITION_WINDOW 20

// установка фгалов и глобальных переменных
bool flag = false;
byte recieved_packet[6];
int close_left = DEFAULT_CLOSE_L;
int close_right = DEFAULT_CLOSE_R;
int open_left = DEFAULT_CLOSE_L-DEFAULT_OPEN;
int open_right = DEFAULT_CLOSE_R+DEFAULT_OPEN;
int close_speed_regulation = -1;
int required_position_l = open_left;
int required_position_r = open_right;
bool position_completed_l = true;
bool position_completed_r = true;

byte form_checksum(byte packet[]){
  byte checksum = (((packet[2] + packet[3] + packet[4] + packet[5] + packet[6]) & 255) | 1) - 1 ;
  return checksum;
  }


void close_speed(int vel = DEFAULT_WORK_SPEED){
  ax12a.setEndless(RIGHT_ID, ON);
  ax12a.setEndless(LEFT_ID, ON);
  ax12a.turn(RIGHT_ID, RIGHT, vel);
  ax12a.turn(LEFT_ID, LEFT, vel);
  }
  
  void send_package(int code, int val1= 0 , int val2 = 0){
    byte s_packet[8];
  s_packet[0] = START_PACKAGE_FLAG;
  s_packet[1] = START_PACKAGE_FLAG;
  s_packet[2] = code;
  s_packet[3] = byte(val1 >> 8);
  s_packet[4] = byte(val1 & 255);
  s_packet[5] = byte(val2 >> 8);
  s_packet[6] = byte(val2 & 255);
  s_packet[7] = form_checksum(s_packet);
  for (int i = 0; i < 8; i++){
    Serial.write(s_packet[i]);
    }
  }
    
  int last_delta_Pos = 0; 
  long I_sum_long = 0;
  unsigned long last_micros = micros();
  
//Функция регулятора закрытия с заданным усилием
void regulator(){
  unsigned long new_last_micros = micros();
  unsigned long delta_t = new_last_micros-last_micros;
  last_micros =  new_last_micros;
  
  int max_I = close_speed_regulation;
  
  int current_position_f = ax12a.readPosition(RIGHT_ID);
  int current_position_s = ax12a.readPosition(LEFT_ID);
  if (current_position_f  < 0 || current_position_s < 0) return;
 // send_package(241, current_position_f,current_position_s);
  int delta_Pos = current_position_f + current_position_s - 1023;
  I_sum_long =I_sum_long+ long((delta_Pos+last_delta_Pos)/2*delta_t);

 if (I_sum_long > max_I*1000000){
    I_sum_long = max_I*1000000;
    }
    if (I_sum_long < -max_I*1000000){
      I_sum_long = -max_I*1000000; 
      }

      
  int P_sum = delta_Pos;
  int I_sum = (int)(I_sum_long/1000000);
  int D_sum = (int)((delta_Pos-last_delta_Pos)*long(1000000/delta_t));

  last_delta_Pos = delta_Pos;
   
  int u  = 5*P_sum+7*I_sum+D_sum/30;
  int val_left =  close_speed_regulation-u;
  int val_right=  close_speed_regulation+ u;

  // ограничение значений
  if (abs(val_right) > 1023) val_right = 1023*((val_right > 0) - (val_right < 0));
  if (abs(val_left) > 1023) val_left = 1023*((val_left > 0) - (val_left < 0));
  
  if (val_right > 0) {
    ax12a.turn(RIGHT_ID, RIGHT, val_right);
    }else{
      ax12a.turn(RIGHT_ID, LEFT, -val_right);
      }
  if (val_left > 0) {
  ax12a.turn(LEFT_ID, LEFT, val_left);
  }else{
    ax12a.turn(LEFT_ID, RIGHT, -val_left);
    }
  }


//Простые функции открытия-закрытия
void close_gripper(){
  ax12a.moveSpeed(RIGHT_ID, close_right, DEFAULT_WORK_SPEED);
  ax12a.moveSpeed(LEFT_ID, close_left, DEFAULT_WORK_SPEED);
}
  
 void open_gripper(){
  ax12a.moveSpeed(RIGHT_ID, open_right, DEFAULT_WORK_SPEED);
  ax12a.moveSpeed(LEFT_ID, open_left, DEFAULT_WORK_SPEED);
  }

// Отладочные функции
void move_right(int angle = 0, int vel = DEFAULT_WORK_SPEED){
  ax12a.moveSpeed(RIGHT_ID, close_right + angle, vel);
  }

void move_left(int angle = 0, int vel = DEFAULT_WORK_SPEED){
  ax12a.moveSpeed(LEFT_ID, close_left - angle, vel);
  }
  
void move_anlge_speed(int angle = DEFAULT_OPEN, int vel = DEFAULT_WORK_SPEED){
  move_right(angle, vel);
  move_left(angle, vel);
  }

// Отправка параметров динамикселей

void back_position_packet(){
  int current_position_r = ax12a.readPosition(RIGHT_ID);
  int current_position_l = ax12a.readPosition(LEFT_ID);
  send_package(BACK_GET_POS,current_position_l,current_position_r);
  }
  
void back_load_packet(){
  int load_r = ax12a.readLoad(RIGHT_ID);
  int load_l = ax12a.readLoad(LEFT_ID);
  send_package(BACK_GET_LOAD,load_l,load_r);
  }

void back_voltage_packet(){
  int voltage_r = ax12a.readVoltage(RIGHT_ID);
  int voltage_l = ax12a.readVoltage(LEFT_ID);
  send_package(BACK_GET_VOLT,voltage_l,voltage_r);
  }
  
void back_temp_packet(){
  int temp_r = ax12a.readTemperature(RIGHT_ID);
  int temp_l = ax12a.readTemperature(LEFT_ID);
  send_package(BACK_GET_TEMP,temp_l,temp_r);
  }
  
  
void back_complete_packet(){
  position_completed_r = check_right_position();
  position_completed_l = check_left_position();
  send_package(BACK_GET_COMP_MOVE,position_completed_l,position_completed_r);
  }


bool check_left_position(){
   int current_position_l = ax12a.readPosition(LEFT_ID);
   if (current_position_l == -1) return false;
   return  ((required_position_l - ERROR_CHECK_POSITION_WINDOW) <= current_position_l )&& (current_position_l <= (required_position_l + ERROR_CHECK_POSITION_WINDOW));
  }
  
bool check_right_position(){
  int current_position_r = ax12a.readPosition(RIGHT_ID);
  if (current_position_r == -1) return false;
  return  ((required_position_r - ERROR_CHECK_POSITION_WINDOW)<=current_position_r) && (current_position_r <= (required_position_r + ERROR_CHECK_POSITION_WINDOW));
  }
  
void setup() {
  Serial.begin(57600);
  ax12a.begin(BaudRate, DirectionPin, &Serial1);
  ax12a.setEndless(RIGHT_ID, OFF);
  ax12a.setEndless(LEFT_ID, OFF);
  
//  int test1 = ax12a.readPosition(RIGHT_ID);
//  int test2 = ax12a.readPosition(LEFT_ID);
//  Serial.print(test1);
//  Serial.print(" ");
//  Serial.print(test2);
//  open_gripper(WORK_SPEED);
//  delay(2000);
//  close_gripper(WORK_SPEED);
//  delay(2000);
}

void loop() {
  if(Serial.available() >= 8){
    byte first_checkbit = Serial.read();
    if (first_checkbit == START_PACKAGE_FLAG){
      byte second_checkbit = Serial.read();
      if (second_checkbit == START_PACKAGE_FLAG){
          for(int i = 0; i<6; i++){
              recieved_packet[i] = Serial.read();
            }    
          byte checksum = (((recieved_packet[0] + recieved_packet[1] + recieved_packet[2] + recieved_packet[3] + recieved_packet[4]) & 255) | 1) - 1 ;
          if(checksum == recieved_packet[5]){
              choose_control(recieved_packet);
          }
      }
    }
  } 
  if (!(position_completed_l && position_completed_r)){
        position_completed_l = check_right_position();
        position_completed_r = check_left_position();
        if (position_completed_l && position_completed_r){
          back_complete_packet();
          } 
    }
    
  if (close_speed_regulation >= 0){
      regulator();
  } 
}
void releasem(){
          close_speed_regulation = -1;
          ax12a.setEndless(RIGHT_ID, ON);
          ax12a.setEndless(LEFT_ID, ON);
  }

  void unreleasem(){
          close_speed_regulation = -1;
          ax12a.setEndless(RIGHT_ID, OFF);
          ax12a.setEndless(LEFT_ID, OFF);
          ax12a.move(RIGHT_ID,ax12a.readPosition(RIGHT_ID));
          ax12a.move(LEFT_ID,ax12a.readPosition(LEFT_ID));
  }
  
  void reset_completing(){
    position_completed_l = false;
    position_completed_r = false;
    }
    
void choose_control(byte packet[6]){
  switch(packet[0]){
    
    case SEND_GET_POS:
          back_position_packet();
          break;
    case SEND_GET_LOAD:
          back_load_packet();
          break;
    case SEND_GET_VOLT:
          back_voltage_packet();
          break;
    case SEND_GET_TEMP:
          back_temp_packet();
          break;
    case SEND_GET_COMP_MOVE:
          back_complete_packet();
          break;
          
    case RELEASE:
          ax12a.turn(LEFT_ID, LEFT, 0);
          ax12a.turn(RIGHT_ID, RIGHT, 0);
          releasem();
          break;
          
    case UNRELEASE:
          unreleasem();
          break;
          
    case SEND_OPEN:
          required_position_l = open_left;
          required_position_r = open_right;
          reset_completing();
          unreleasem();
          open_gripper();
          break;
    case SEND_CLOSE: 
          required_position_l = close_left;
          required_position_r = close_right;
          reset_completing();
          unreleasem();
          close_gripper();
          break;
    case SEND_CLOSE_SPEED:
          required_position_l = close_left;
          required_position_r = close_right;
          reset_completing();
          releasem();
          close_speed_regulation = int((packet[1] << 8) + packet[2]);
          I_sum_long = 0;
          last_micros = micros();
          break;
    case SEND_OPEN_SPEED:
          required_position_l = open_left;
          required_position_r = open_right;
          reset_completing();
          unreleasem();
          move_anlge_speed(DEFAULT_OPEN, int((packet[1] << 8) + packet[2]));
          break;
          
          //дополнительные и отладочные функци
    case SEND_ANGLE:
          reset_completing();
          unreleasem();
          move_anlge_speed(int((packet[3] << 8) + packet[4]) ,DEFAULT_WORK_SPEED);
          break;
   
    case SEND_LEFT_ANGLE:
          reset_completing();
          unreleasem();
          move_left(int((packet[3] << 8) + packet[4]) ,DEFAULT_WORK_SPEED);
          break;

    case SEND_RIGHT_ANGLE :
          reset_completing();
          unreleasem();
          move_right(int((packet[3] << 8) + packet[4]) ,DEFAULT_WORK_SPEED);
          break;

    case SEND_ANGLE_SPEED :
          reset_completing();
          unreleasem();
          move_anlge_speed(int((packet[3] << 8) + packet[4]) ,int((packet[1] << 8) + packet[2]));
          break;
          
    case SEND_LEFT_ANGLE_SPEED :
          reset_completing();
          unreleasem();
          move_left(int((packet[3] << 8) + packet[4]) ,int((packet[1] << 8) + packet[2]));
          break;

    case SEND_RIGHT_ANGLE_SPEED :
          reset_completing();
          unreleasem();
          move_right(int((packet[3] << 8) + packet[4]) ,int((packet[1] << 8) + packet[2]));
          break;
    }
  }
