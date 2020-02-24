//  Syed Rehman
//  Quadcopter flight controller

#include <Wire.h>
#include <Servo.h>
#include <PID_v1.h>

double rad_to_degree = (180 / 3.141592654);

//~~~~~~~RC in this range means no movements~~~~~~~~~~~~ upper mean upper limit, lower = lower limit
#define MAX_PITCH      1990
#define MAX_ROLL       1975
#define MAX_YAW        1980
#define MIN_PITCH       990
#define MIN_ROLL        985
#define MIN_YAW         995
#define NO_PITCH_UPPER 1494
#define NO_PITCH_LOWER 1480
#define NO_ROLL_UPPER  1480
#define NO_ROLL_LOWER  1466
#define NO_YAW_UPPER   1494
#define NO_YAW_LOWER   1484
#define NO_POWER       1000
#define MAX_POWER      1850

//~~~~~~~motors~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Servo right_front, right_back, left_front, left_back;
float right_front_speed, right_back_speed, left_front_speed, left_back_speed;

//~~~~~~~mpu 6050 variables~~~~~~~~~~~~~~~~
int gyro_x, gyro_y, gyro_z;
int acc_x, acc_y, acc_z, acc_total_vector;
long gyro_x_cal, gyro_y_cal, gyro_z_cal;
long loop_timer;
float angle_pitch_gyro, angle_roll_gyro;
float angle_roll_acc, angle_pitch_acc;
double current_pitch, current_roll;

//~~~~~~~rc receiver variables~~~~~~~~~~~~~
double desired_pitch, desired_roll, desired_yaw, throttle;

//~~~~~~~PID Controller variables~~~~~~~~~~
#define KP    1 //3.2
#define KI  .05 //0.006
#define KD  .25 //1.4
float p_roll = 0;
float i_roll = 0;
float d_roll = 0;
float pid_roll = 0;
float roll_error = 0;
float roll_error_prev = 0;
float p_pitch = 0;
float i_pitch = 0;
float d_pitch = 0;
float pid_pitch = 0;
float pitch_error = 0;
float pitch_error_prev = 0;
double elapsedTime;

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

double roll_output, pitch_output;
PID roll(&current_roll, &roll_output, &desired_roll, KP, KI, KD, DIRECT);
PID pitch(&current_pitch, &pitch_output, &desired_pitch, KP, KI, KD, DIRECT);
int print_counter = 0;

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

void setup_rc_reciever(){
  //setting up pins for rc receiver 4 channels
  pinMode(8, INPUT);
  pinMode(7, INPUT);
  pinMode(6, INPUT);
  pinMode(4, INPUT);
}

void setup_motors(){
  right_front.attach(10, 1000, 2000);
  right_back.attach(9, 1000, 2000);
  left_front.attach(5, 1000, 2000);
  left_back.attach(3, 1000, 2000);
  
}

void setup_mpu_6050_registers(){
  Wire.begin();  
  //Activate the MPU-6050
  Wire.beginTransmission(0x68); 
  Wire.write(0x6B); 
  Wire.write(0x00);  
  Wire.endTransmission(); 
  //Configure the accelerometer (+/-8g)
  Wire.beginTransmission(0x68);  
  Wire.write(0x1C);  
  Wire.write(0x10);  
  Wire.endTransmission(); 
  //Configure the gyro (500dps full scale)
  Wire.beginTransmission(0x68); 
  Wire.write(0x1B);  
  Wire.write(0x08);  
  Wire.endTransmission();

  //Run this code 4 sec for calibration.
  for (int i = 0; i < 250 ; i++){   
    read_mpu_6050_data();                           
    gyro_x_cal += gyro_x;                           
    gyro_y_cal += gyro_y;                           
    gyro_z_cal += gyro_z;                           
    delay(3);                                       
  }
  gyro_x_cal /= 250;  //get averages
  gyro_y_cal /= 250;  
  gyro_z_cal /= 250; 
}

void read_mpu_6050_data(){ 
  //Subroutine for reading the raw gyro , accelerometer data and temperature
  Wire.beginTransmission(0x68);  //Start communicating with the MPU-6050
  Wire.write(0x3B);  //Send the requested starting register
  Wire.endTransmission(false); 
  Wire.requestFrom(0x68,6,true);  //Request 6 bytes from the MPU-6050
  acc_x = Wire.read()<<8|Wire.read();    //Add the low and high byte to the acc_* variable
  acc_y = Wire.read()<<8|Wire.read();    
  acc_z = Wire.read()<<8|Wire.read();    


  Wire.beginTransmission(0x68);
  Wire.write(0x43);     //Gyro data first address
  Wire.endTransmission(false);
  Wire.requestFrom(0x68,4,true); //Just 4 registers
   
  gyro_x = Wire.read()<<8|Wire.read();   //Add the low and high byte to the gyro_* variable
  gyro_y = Wire.read()<<8|Wire.read();   
  gyro_z = Wire.read()<<8|Wire.read();  

}

void get_current_mpu_values(){
  read_mpu_6050_data();
  angle_roll_acc = atan((acc_y /16384.0)/sqrt(pow((acc_x/16384.0),2) + pow((acc_z/16384.0),2)))*rad_to_degree;  //Euler's equation
  angle_pitch_acc = atan(-1*(acc_x/16384.0)/sqrt(pow((acc_y/16384.0),2) + pow((acc_z/16384.0),2)))*rad_to_degree;

  angle_roll_gyro = gyro_x / 131.0;
  angle_pitch_gyro = gyro_y / 131.0;

  current_roll = 0.98 *(current_roll + angle_roll_gyro * elapsedTime) + 0.02* angle_roll_acc;
  current_pitch = 0.98 *(current_pitch + angle_pitch_gyro * elapsedTime) + 0.02* angle_pitch_acc;
}

void get_desired_values(){
  desired_roll = pulseIn(8, HIGH);    //A0, roll
  desired_pitch = pulseIn(7, HIGH);   //A1, pitch 
  throttle = pulseIn(6, HIGH);        //A2, power
  desired_yaw = pulseIn(4, HIGH);     //A3, yaw

  if(throttle < NO_POWER){
    throttle = 1000;
  }else{
    if(throttle > MAX_POWER){
      throttle = MAX_POWER;
    }

    if(NO_ROLL_LOWER < desired_roll < NO_ROLL_UPPER){
      desired_roll = 0;
    }else{
      desired_roll = map(desired_roll, MIN_ROLL, MAX_ROLL, -30, 30);
    }
    
    if(NO_PITCH_LOWER < desired_pitch < NO_PITCH_UPPER){
      desired_pitch = 0;
    }else{
      desired_pitch = map(desired_pitch, MIN_PITCH, MAX_PITCH, -30, 30);
    }
    
    if((NO_YAW_LOWER < desired_yaw) &&  (desired_yaw < NO_YAW_UPPER)){
      desired_yaw = 0;
    }else{
      desired_yaw = map(desired_yaw, MIN_YAW, MAX_YAW, -5, 5);
    }
  }
}

void get_pid(){
    
  roll_error = current_roll - desired_roll;
  pitch_error = current_pitch - desired_pitch;
  
  p_roll = KP * roll_error;
  p_pitch = KP * pitch_error;

  if(-3 < roll_error < 3){
    i_roll = i_roll + (KI * roll_error);  
  }
   if(-3 < pitch_error < 3){
    i_pitch = i_pitch + (KI * pitch_error);  
  }
  
  d_roll = KD *((roll_error - roll_error_prev) / elapsedTime);
  d_pitch = KD *((pitch_error - pitch_error_prev) / elapsedTime);
  
  /*The final PID values is the sum of each of this 3 parts*/
  pid_roll = p_roll + i_roll + d_roll;
  pid_pitch = p_pitch + i_pitch + d_pitch;

  if(pid_roll < -1000){
    pid_roll = -1000;
  }
  if(pid_roll > 1000){
    pid_roll = 1000;
  }
  if(pid_pitch < -1000){
    pid_pitch = -1000;
  }
  if(pid_pitch > 1000){
    pid_pitch = 1000;
  }
 
  // roll control. 
  left_front_speed = throttle + pid_roll;
  left_back_speed = throttle + pid_roll;
  right_front_speed = throttle - pid_roll;
  right_back_speed = throttle - pid_roll;

  // pitch control
  left_front_speed = throttle + pid_pitch;
  right_front_speed = throttle + pid_pitch;
  left_back_speed = throttle - pid_pitch;
  right_back_speed = throttle - pid_pitch;
  

  if(left_front_speed < 1020){
    left_front_speed = 1020;
  }
  if(left_front_speed > 1850){
    left_front_speed = 1850;
  }
  if(right_front_speed < 1020){
    right_front_speed = 1020;
  }
  if(right_front_speed > 1850){
    right_front_speed = 1850;
  }
  if(left_back_speed < 1020){
    left_back_speed = 1020;
  }
  if(left_back_speed > 1850){
    left_back_speed = 1850;
  }
  if(right_back_speed < 1020){
    right_back_speed = 1020;
  }
  if(right_back_speed > 1850){
    right_back_speed = 1850;
  }
}

void get_pid_2(){
  roll.Compute(); //roll_output
  pitch.Compute(); //pitch_output
  // roll control. 
  
  
  if(left_front_speed < 1020){
    left_front_speed = NO_POWER;
  }
  if(left_front_speed > 1850){
    left_front_speed = 1850;
  }
  if(right_front_speed < 1020){
    right_front_speed = NO_POWER;
  }
  if(right_front_speed > 1850){
    right_front_speed = 1850;
  }
  if(left_back_speed < 1020){
    left_back_speed = NO_POWER;
  }
  if(left_back_speed > 1850){
    left_back_speed = 1850;
  }
  if(right_back_speed < 1020){
    right_back_speed = NO_POWER;
  }
  if(right_back_speed > 1850){
    right_back_speed = 1850;
  }
}

void print_speed(){
//  Serial.print("throttle:");
//  Serial.print(throttle);
//
//  Serial.print(" roll:");
//  Serial.print(desired_roll);
//
//  Serial.print(" pitch:");
//  Serial.println(desired_pitch);

  if((print_counter % 10) == 0){
//    Serial.print("front left :");
//    Serial.print(left_front_speed);
//    Serial.print("  front right :");
//    Serial.print(right_front_speed);
//    Serial.print("  back left :");
//    Serial.print(left_back_speed);
//    Serial.print("  back right :");
//    Serial.print(right_back_speed);   
    Serial.print(" ~~~ current roll:");
    Serial.print(current_roll);
    Serial.print("  desired roll :");
    Serial.print(desired_roll);
    Serial.print(" ~~~ current pitch:");
    Serial.print(current_pitch);
    Serial.print("  desired pitch :");
    Serial.println(desired_pitch);
  }
  print_counter++;
}

void setup() {
  Serial.begin(250000); 
  setup_rc_reciever();
  setup_mpu_6050_registers();  
  setup_motors();
  roll.SetMode(AUTOMATIC);
  pitch.SetMode(AUTOMATIC);
  loop_timer = micros();  //Reset the loop timer
  Serial.println("setup done");
}

void loop(){
  elapsedTime = (micros() - loop_timer);  //time passed in sec
  get_current_mpu_values(); //figure out current roll and current pitch
  get_desired_values(); //get controller values.
  get_pid_2();
  left_front.writeMicroseconds(left_front_speed);
  left_back.writeMicroseconds(left_back_speed);
  right_front.writeMicroseconds(right_front_speed);
  right_back.writeMicroseconds(right_back_speed);
  
  roll_error_prev = roll_error;
  pitch_error_prev = pitch_error;

  print_speed();
  
  while(micros() - loop_timer < 4000);  //Wait until the loop_timer reaches 4000us (250Hz) before starting the next loop
  loop_timer = micros();   //Reset the loop timer
}
