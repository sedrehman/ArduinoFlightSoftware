//  Syed Rehman
//  Quadcopter flight controller

#include <Wire.h>
#include <Servo.h>

double rad_to_degree = (180 / 3.141592654);
unsigned long loop_timer, elapsed_time;

//~~~~~~~RC in this range means no movements~~~~~~~~~~~~ upper mean upper limit, lower = lower limit
#define MAX_PITCH      2000
#define MAX_ROLL       2000
#define MAX_YAW        2000
#define MIN_PITCH      1000
#define MIN_ROLL       1000
#define MIN_YAW        1000
#define NO_POWER       1050
#define MAX_POWER      1850

//~~~~~~~motors~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Servo right_front, right_back, left_front, left_back;
int right_front_speed, right_back_speed, left_front_speed, left_back_speed;
int right_front_cal, right_back_cal, left_front_cal, left_back_cal;

//~~~~~~~mpu 6050 variables~~~~~~~~~~~~~~~~
int gyro_x, gyro_y, gyro_z, temperature;
int acc_x, acc_y, acc_z;
double acc_pitch_cal, acc_roll_cal;
double gyro_x_cal, gyro_y_cal, gyro_z_cal;
double angle_pitch_gyro, angle_roll_gyro, angle_yaw_gyro;
double angle_roll_acc[4], angle_pitch_acc[4];
double acc_pitch, acc_roll;
double current_pitch, current_roll;
int i = 0;
double mpu_d;
//~~~~~~~rc receiver variables~~~~~~~~~~~~~
volatile int rc_values[4];
byte ch1, ch2, ch3, ch4;
unsigned long timer_1, timer_2, timer_3, timer_4, current_time ;
int throttle, desired_pitch, desired_roll, desired_yaw;

//~~~~~~~pid variables~~~~~~~~~~~~~~~~~~~~~
#define I_CONST 1
#define P_CONST 1
#define D_CONST 1
#define MAX_PITCH_CHANGE 50
#define MIN_PITCH_CHANGE -50
#define MAX_ROLL_CHANGE 50
#define MIN_ROLL_CHANGE -50

double pitch_diff, roll_diff;
double p_pitch, p_roll, i_pitch, i_roll; //p = proportional , i = integral
double i_mem_pitch, i_mem_roll, last_diff_pitch, last_diff_roll;
double pitch_output, roll_output;


void setup() {
  Serial.begin(9600);
  Wire.begin();
  TWBR = 12;
  current_roll = 0;
  current_pitch = 0;
  setup_mpu_6050_registers();  

  i_mem_pitch = 0;
  i_mem_roll = 0;
  last_diff_pitch = 0;
  last_diff_roll = 0;
  
  /*
  pinMode(8, INPUT);   PCINT0    roll
  pinMode(9, INPUT);   PCINT2   pitch
  pinMode(10, INPUT);   PCINT3   throttle
  pinMode(11, INPUT);   PCINT4   yaw
  */
  cli();
  PCICR |= (1 << PCIE0);
  PCMSK0 |= (1 << PCINT0);
  PCMSK0 |= (1 << PCINT1);
  PCMSK0 |= (1 << PCINT2);
  PCMSK0 |= (1 << PCINT3);
  sei();

  angle_pitch_gyro = 0.0;
  angle_roll_gyro = 0.0;

  setup_motors();

  right_front_speed = 1000;
  right_back_speed = 1000;
  left_front_speed = 1000;
  left_back_speed = 1000;
  
  loop_timer = micros();  //Reset the loop timer
  Serial.println("setup done");
}


void loop(){
  //elapsed_time = (micros() - loop_timer);
  read_mpu_6050_data();
  gyro_x -= gyro_x_cal;
  gyro_y -= gyro_y_cal;
  gyro_z -= gyro_z_cal;

  angle_pitch_acc[0] = angle_pitch_acc[1];
  angle_pitch_acc[1] = angle_pitch_acc[2];
  angle_pitch_acc[2] = angle_pitch_acc[3];
  angle_pitch_acc[3] = atan((acc_y /16384.0)/sqrt(pow((acc_x/16384.0),2) + pow((acc_z/16384.0),2)))*rad_to_degree - acc_pitch_cal;  //Euler's equation
  acc_pitch = (angle_pitch_acc[0] + angle_pitch_acc[1] + angle_pitch_acc[2] + angle_pitch_acc[3]) / 4;  //using rolling avg

  angle_roll_acc[0] = angle_roll_acc[1];
  angle_roll_acc[1] = angle_roll_acc[2];
  angle_roll_acc[2] = angle_roll_acc[3];
  angle_roll_acc[3] = atan(-1*(acc_x/16384.0)/sqrt(pow((acc_y/16384.0),2) + pow((acc_z/16384.0),2)))*rad_to_degree - acc_roll_cal;
  acc_roll = (angle_roll_acc[0] + angle_roll_acc[1] + angle_roll_acc[2] + angle_roll_acc[3]) / 4;

  //mpu_d = 1 / (
  angle_pitch_gyro += (gyro_x *0.000346);  
  angle_roll_gyro += (gyro_y *0.000346);   
  angle_yaw_gyro += (gyro_z *0.000346);  //.00048 @ 125hz

  current_roll = (0.999 *angle_roll_gyro) + (0.001* acc_roll);
  current_pitch = (0.999 *angle_pitch_gyro) + (0.001* acc_pitch);

  if(throttle == 1000 || ((current_pitch < 2 && current_pitch > -2) && (acc_pitch > -.02 && acc_pitch < .02))){
    angle_pitch_gyro = acc_pitch;
    current_pitch = acc_pitch;
  }
  if(throttle == 1000 || ((current_roll < 2 && current_roll > -2) && (acc_roll > -.02 && acc_roll < .02))){
    angle_roll_gyro = acc_roll;
    current_roll = acc_roll;
  }

  //If the IMU has yawed transfer the roll angle to the pitch angle
  current_pitch += current_roll * sin(gyro_z * 0.000001066);
  //If the IMU has yawed transfer the pitch angle to the roll angle               
  current_roll -= current_pitch * sin(gyro_z * 0.000001066);        
  
  get_desired_values();
  
  if(throttle < 1050){
    right_front_speed = 1000;
    right_back_speed = 1000;
    left_front_speed = 1000;
    left_back_speed = 1000;
    Serial.print("~");
  }else{
    get_pid();
    Serial.print(current_roll);
    Serial.print("  output:");
    Serial.println(left_front_speed);
  }
  left_front.writeMicroseconds(left_front_speed);
  left_back.writeMicroseconds(left_back_speed);
  right_front.writeMicroseconds(right_front_speed);
  right_back.writeMicroseconds(right_back_speed);
 


  
// Serial.println(elapsed_time);
  while(elapsed_time < 8000){
    elapsed_time = micros() - loop_timer;
  }
  loop_timer = micros();
}

//###########################################################################################################
void get_pid(){
  right_front_speed = throttle;
  right_back_speed = throttle;
  left_front_speed = throttle;
  left_back_speed = throttle;
  //~~~~~~~~~~~~~~~~~~~~~~~~~~pitch ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  pitch_diff = current_pitch - desired_pitch;
  i_mem_pitch += (I_CONST * pitch_diff);
  
  if( i_mem_pitch > MAX_PITCH_CHANGE){
    i_mem_pitch = MAX_PITCH_CHANGE;
  }
  if( i_mem_pitch < MIN_PITCH_CHANGE ){
    i_mem_pitch = MIN_PITCH_CHANGE;
  }
  pitch_output = (P_CONST * pitch_diff) + (D_CONST * (pitch_diff - last_diff_pitch)) + i_mem_pitch;
  last_diff_pitch = pitch_diff;

  //~~~~~~~~~~~~~~~~~~~~~~~~~roll ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  roll_diff = desired_roll - current_roll;
  i_mem_roll += (I_CONST * roll_diff);
  
  if( i_mem_roll > MAX_ROLL_CHANGE){
    i_mem_roll = MAX_ROLL_CHANGE;
  }
  if( i_mem_roll < MIN_ROLL_CHANGE ){
    i_mem_roll = MIN_ROLL_CHANGE;
  }
  roll_output = (P_CONST * roll_diff) + (D_CONST * (roll_diff - last_diff_roll)) + i_mem_roll;
  last_diff_roll = roll_diff;

  //+++++++++++++++++++++++++++++++++++++++motor control++++++++++++++++++++++++++++++++++++++++++++++++++++
  if(pitch_output > 2){
    right_back_speed = throttle + pitch_output;
    left_back_speed = throttle + pitch_output;
    if(right_back_speed > MAX_PITCH){ right_back_speed = MAX_PITCH; }
    if(left_back_speed > MAX_PITCH){ left_back_speed = MAX_PITCH; }
  }
  if(pitch_output < -2){
    right_front_speed = throttle + (-1* pitch_output);
    left_front_speed = throttle + (-1* pitch_output);
    if(right_front_speed > MAX_PITCH){ right_front_speed = MAX_PITCH; }
    if(left_front_speed > MAX_PITCH){ left_front_speed = MAX_PITCH; }
  }

  if(roll_output < -2){
    right_front_speed = throttle + (-1* roll_output);
    right_back_speed = throttle + (-1* roll_output);
    if(right_front_speed > MAX_PITCH){ right_front_speed = MAX_PITCH; }
    if(right_back_speed > MAX_PITCH){ right_back_speed = MAX_PITCH; }
  }
  if(roll_output > 2){
    left_front_speed = throttle + roll_output;
    left_back_speed = throttle + roll_output;
    if(left_front_speed > MAX_PITCH){ left_front_speed = MAX_PITCH; }
    if(left_back_speed > MAX_PITCH){ left_back_speed = MAX_PITCH; }
  }
}
//###########################################################################################################
/*
pinMode(8, INPUT);   PCINT0    roll
pinMode(9, INPUT);   PCINT23   pitch
pinMode(10, INPUT);   PCINT22   throttle
pinMode(11, INPUT);   PCINT20   yaw
*/
ISR(PCINT0_vect){
  current_time = micros();
  if(PINB & B00000001){  
    if(ch1 == 0){         
      ch1 = 1;                 
      timer_1 = current_time;             
    }
  }
  else if(ch1 == 1){   
    ch1 = 0;                           
    rc_values[0] = current_time - timer_1;   
  } 

  
  if(PINB & B00000010){  
    if(ch2 == 0){ 
      ch2 = 1;  
      timer_2 = current_time; 
    }
  }
  else if(ch2 == 1){     
    ch2 = 0; 
    rc_values[1] = current_time - timer_2;
  }

  
  if(PINB & B00000100){  
    if(ch3 == 0){
      ch3 = 1; 
      timer_3 = current_time; 
    }
  }
  else if(ch3 == 1){ 
    ch3 = 0;   
    rc_values[2] = current_time - timer_3;  
  }
  
  
  if(PINB & B00001000){ 
    if(ch4 == 0){     
      ch4 = 1;   
      timer_4 = current_time; 
    }
  }
  else if(ch4 == 1){   
    ch4 = 0; 
    rc_values[3] = current_time - timer_4;
  } 
  
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
  Wire.requestFrom(0x68,6,true); 
   
  gyro_x = Wire.read()<<8|Wire.read();   //Add the low and high byte to the gyro_* variable
  gyro_y = Wire.read()<<8|Wire.read();   
  gyro_z = Wire.read()<<8|Wire.read();  
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

  //accel calibration.
  for (i = 0; i < 800 ; i++){   
    read_mpu_6050_data();                           
    acc_pitch_cal += atan((acc_y /16384.0)/sqrt(pow((acc_x/16384.0),2) + pow((acc_z/16384.0),2)))*rad_to_degree ;  //Euler's equation
    acc_roll_cal += atan(-1*(acc_x/16384.0)/sqrt(pow((acc_y/16384.0),2) + pow((acc_z/16384.0),2)))*rad_to_degree ;                         
    delayMicroseconds(8000);                                     
  }
  acc_pitch_cal /= 800;
  acc_roll_cal /= 800;
  Serial.print("accel cal pitch:");
  Serial.print(acc_pitch_cal);
  Serial.print("  roll:");
  Serial.print(acc_roll_cal);

  //gyro calibration.
  for (i = 0; i < 800 ; i++){   
    read_mpu_6050_data();                           
    gyro_x_cal += gyro_x;                           
    gyro_y_cal += gyro_y;                           
    gyro_z_cal += gyro_z;                           
    delayMicroseconds(8000);                                     
  }
  gyro_x_cal /= 800;  //get averages
  gyro_y_cal /= 800;  
  gyro_z_cal /= 800;
}

void get_desired_values(){
  desired_yaw = rc_values[0];
  throttle = rc_values[1];
  desired_pitch = rc_values[2];
  desired_roll = rc_values[3];
  
  if(throttle <= NO_POWER){
    throttle = 1000;
    desired_roll = 0;
    desired_pitch = 0;
    desired_yaw = 0;
  }else{
    if(throttle > MAX_POWER){
      throttle = MAX_POWER;
    }
    desired_roll = map(desired_roll, MIN_ROLL, MAX_ROLL, -30, 30);
    desired_pitch = map(desired_pitch, MIN_PITCH, MAX_PITCH, -30, 30);
    desired_yaw = map(desired_yaw, MIN_YAW, MAX_YAW, -30, 30);
  }
}

void setup_motors(){
  right_front.attach(7, 1000, 2000);
  right_back.attach(6, 1000, 2000);
  left_front.attach(5, 1000, 2000);
  left_back.attach(3, 1000, 2000);
}
