//  Syed Rehman
//  Quadcopter flight controller

#include <Wire.h>
#include <Servo.h>

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
#define NO_POWER       1010
#define MAX_POWER      1850

//~~~~~~~motors~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Servo right_front, right_back, left_front, left_back;
float right_front_speed, right_back_speed, left_front_speed, left_back_speed;

//~~~~~~~mpu 6050 variables~~~~~~~~~~~~~~~~
int gyro_x, gyro_y, gyro_z;
int acc_x, acc_y, acc_z, acc_total_vector;
long gyro_x_cal, gyro_y_cal, gyro_z_cal;
long loop_timer;
float angle_pitch_gyro, angle_roll_gyro, angle_yaw_gyro;
float angle_roll_acc, angle_pitch_acc;
double current_pitch, current_roll;

//~~~~~~~rc receiver variables~~~~~~~~~~~~~
double desired_pitch, desired_roll, desired_yaw, throttle;

//~~~~~~~pid variables~~~~~~~~~~~~~~~~~~~~~

double pitch_diff, roll_diff;
double p_pitch, p_roll, i_pitch, i_roll; //p = proportional , i = integral
double i_mem_pitch, i_mem_roll, last_diff_pitch, last_diff_roll;

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
    //gyro_z_cal += gyro_z;                           
    delay(3);                                       
  }
  gyro_x_cal /= 250;  //get averages
  gyro_y_cal /= 250;  
  //gyro_z_cal /= 250; 
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

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


void get_current_mpu_values(){
  read_mpu_6050_data();
  angle_pitch_acc = atan((acc_y /16384.0)/sqrt(pow((acc_x/16384.0),2) + pow((acc_z/16384.0),2)))*rad_to_degree;  //Euler's equation
  angle_roll_acc = atan(-1*(acc_x/16384.0)/sqrt(pow((acc_y/16384.0),2) + pow((acc_z/16384.0),2)))*rad_to_degree;

  angle_pitch_gyro = (gyro_x / 131.0) *(.004);
  angle_roll_gyro = (gyro_y / 131.0) * (.004);
  angle_yaw_gyro = (gyro_z / 131.0) * (.004);

  current_roll = 0.98 *(current_roll + angle_roll_gyro * elapsedTime) + 0.02* angle_roll_acc;
  current_pitch = 0.98 *(current_pitch + angle_pitch_gyro * elapsedTime) + 0.02* angle_pitch_acc;
}

void get_desired_values(){
  throttle = pulseIn(6, HIGH);        //A2, power
 
  if(throttle <= NO_POWER){
    throttle = 1000;
    desired_roll = 0;
    desired_pitch = 0;
    desired_yaw = 0;
  }else{
    
    desired_roll = pulseIn(8, HIGH);    //A0, roll
    desired_pitch = pulseIn(7, HIGH);   //A1, pitch 
    desired_yaw = pulseIn(4, HIGH);     //A3, yaw

    if(throttle > MAX_POWER){
      throttle = MAX_POWER;
    }

    if((NO_ROLL_LOWER < desired_roll) && ( desired_roll < NO_ROLL_UPPER )){
      desired_roll = 0;
    }else{
      desired_roll = map(desired_roll, MIN_ROLL, MAX_ROLL, -30, 30);
    }
    
    if((NO_PITCH_LOWER < desired_pitch) && (desired_pitch < NO_PITCH_UPPER)){
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
//###########################################################################################################
void get_pid(){
  //current roll --- desired roll
  //current pitch --- desired pitch
  //pitch_diff, roll_diff;

  pitch_diff = desired_pitch - current_pitch;
  roll_diff = desired_roll - current_roll;

  i_mem_pitch += (
}
//###########################################################################################################

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
  get_pid();
//  left_front.writeMicroseconds(left_front_speed);
//  left_back.writeMicroseconds(left_back_speed);
//  right_front.writeMicroseconds(right_front_speed);
//  right_back.writeMicroseconds(right_back_speed);

  left_front.writeMicroseconds(throttle);
  left_back.writeMicroseconds(throttle);
  right_front.writeMicroseconds(throttle);
  right_back.writeMicroseconds(throttle);
  
  roll_error_prev = roll_error;
  pitch_error_prev = pitch_error;

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  Serial.print("current roll:");
  Serial.print(current_roll);
  Serial.print(" desired roll:");
  Serial.print(desired_roll);
  
  Serial.print(" ~~~current pitch:");
  Serial.print(current_pitch);
  Serial.print(" desired pitch:");
  Serial.print(desired_pitch); 
  Serial.print("\n");
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  
  while(micros() - loop_timer < 4000);  //Wait until the loop_timer reaches 4000us (250Hz) before starting the next loop
  loop_timer = micros();   //Reset the loop timer
}
