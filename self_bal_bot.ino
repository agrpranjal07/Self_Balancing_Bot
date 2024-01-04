 #include <Wire.h>
const int MPU6050_ADDRESS = 0x68; 

unsigned long lastTime;


float acc_angle, gyro_rate,prevAngle=0; // Calculated tilt angle and angular velocity
float MIN_ABS_SPEED=127.5; // Motor speed adjustment
int a=50;
int sampleTime = 10; //0.005 sec
float p=1000/sampleTime;
float pitchSetpoint = -0.8,yawSetpoint = -10.0; // Desired tilt angle (setpoint) 
float Kp_p = 40; // Proportional gain
float Ki_p = 0.8; // Integral gain
float Kd_p = 98; // Derivative gain
float Kp_y = 12; // Proportional gain
float Ki_y = 0.5; // Integral gain
float Kd_y = 38; // Derivative gain

float prev_error = 0;
float error,integral = 0,derivative;

const int enableMotor1=5;
const int motorPin1 = 6;
const int motorPin2 = 7;
const int motorPin3 = 8;
const int motorPin4 = 9;
const int enableMotor2=10;
float acc[3],gyr[3];

void setup() {
  Serial.begin(9600);  
  Wire.begin();        
  initializeMPU6050(); 
  
}

void loop() {
  // Read data from MPU6050
  int16_t accData[3]; 
  int16_t gyroData[3];     

  readMPU6050Data(accData, gyroData);
    for(int i=0;i<3;i++){
    acc[i]=accData[i]/16384.0;
    gyr[i]=gyroData[i]/131.0;}
 
  acc_angle = atan2(-acc[1], acc[2]) * (180.0/PI); // deg

 
  float gyro_rate_z = atan2(-acc[0], acc[1]) * (180.0/PI); // deg/sec


  float pitch_error= PID(acc_angle, pitchSetpoint,Kp_p,Ki_p,Kd_p);
  float yaw_error= PID(gyro_rate_z, yawSetpoint,Kp_y,Ki_y,Kd_y);

  double speedError_A = pitch_error + yaw_error;
  double speedError_B = pitch_error - yaw_error;


  double speedErrorA = map(speedError_A,-(a*(Kp_p+Kp_y)),(a*(Kp_p+Kp_y)), -255, 255);
  double speedErrorB = map(speedError_B,-(a*(Kp_p-Kp_y)),(a*(Kp_p-Kp_y)), -255, 255);

  double motorSpeed_A = -  speedErrorA;
  double motorSpeed_B = - speedErrorB;

  rotateMotor(motorSpeed_A, motorSpeed_B);
  // Serial.print("Angle: ");
  // Serial.print(acc_angle);
  // Serial.print(" ");
  // Serial.print("Rate: ");
  // Serial.print(gyro_rate_z);
  // Serial.print(" ");
  // Serial.print("Motor A: ");
  // Serial.print(motorSpeed_A);
  // Serial.print(" ");
  // Serial.print("Motor B: ");
  // Serial.print(motorSpeed_B);
  // Serial.print(" ");
  // Serial.print("pitch_error : ");
  // Serial.print(pitch_error);
  // Serial.print(" ");
  // Serial.print("yaw_error : ");
  // Serial.println(yaw_error);
  

  delay(10);
}

void initializeMPU6050() {
  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(0x6B); 
  Wire.write(0);    
  Wire.endTransmission();
}

void readMPU6050Data(int16_t accelData[], int16_t gyrosData[]) {
  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(0x3B); 
  Wire.endTransmission(false); 

  Wire.requestFrom(MPU6050_ADDRESS, 14, true);

  accelData[0] = Wire.read() << 8 | Wire.read(); // accH and AccL
  accelData[1] = Wire.read() << 8 | Wire.read(); // accH and accL
  accelData[2] = Wire.read() << 8 | Wire.read(); // accH and accL
  int16_t a= Wire.read()<< 8| Wire.read();      // To skip temp sensor
  gyrosData[0] = Wire.read() << 8 | Wire.read(); // gyroH and gyroL
  gyrosData[1] = Wire.read() << 8 | Wire.read(); // gyroH and gyroL
  gyrosData[2] = Wire.read() << 8 | Wire.read(); // gyroH and gyroL
}


int PID(float currentAngle, float setpoint,float Kp,float Ki,float Kd){
  unsigned long now = millis();
  int timeChange = (now - lastTime);
  if(timeChange>=sampleTime){
  
  error = currentAngle-setpoint;
  integral += Ki *error/p;
   if (integral > 400) integral = 400;
   else if (integral < -400) integral = -400;
  derivative =p* (error - prev_error);
  prev_error = error;
  
  }
  // Serial.print("Error: ");
  // Serial.print(error);

  return ((Kp * error) +  (Ki*integral) - (Kd * derivative));
}

void rotateMotor(int speed1, int speed2) {
  digitalWrite(motorPin1, speed1 > 0 ? HIGH : LOW);
  digitalWrite(motorPin2, speed1 > 0 ? LOW : HIGH);

  digitalWrite(motorPin3, speed2 > 0 ? HIGH : LOW);
  digitalWrite(motorPin4, speed2 > 0 ? LOW : HIGH);

  analogWrite(enableMotor1, abs(speed1));
  analogWrite(enableMotor2, abs(speed2));
}
