#include <Wire.h>
const int MPU6050_ADDRESS = 0x68;  
float acc[3],gyr[3];

void setup() {
  Serial.begin(9600);  
  Wire.begin();        
  initializeMPU6050(); 
}

void loop() {
  int16_t accelerometerData[3]; 
  int16_t gyroscopeData[3];     

  
  readMPU6050Data(accelerometerData, gyroscopeData);

  for(int i=0;i<3;i++){
    acc[i]=accelerometerData[i]/16384.0;
    gyr[i]=gyroscopeData[i]/131.0;
  }
  
  Serial.print("Accel (x, y, z): ");
  Serial.print(acc[0]);
  Serial.print(", ");
  Serial.print(acc[1]);
  Serial.print(", ");
  Serial.print(acc[2]);
  Serial.print(", ");
  Serial.print("pitchSetpoint: ");
  Serial.print(atan2(-acc[1], acc[2]) * (180.0/PI));
  Serial.print(", ");
  Serial.print(" | Gyro (x, y, z): ");
  Serial.print(gyr[0]);
  Serial.print(", ");
  Serial.print(gyr[1]);
  Serial.print(", ");
  Serial.println(gyr[2]);
  Serial.print("yawSetpoint: ");
  Serial.println(atan2(-acc[0], acc[1]) * (180.0/PI));
  delay(1000); 
}

void initializeMPU6050() {
  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(0x6B); 
  Wire.write(0);    
  Wire.endTransmission();
}

void readMPU6050Data(int16_t accelData[], int16_t gyroData[]) {
  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(0x3B); 
  Wire.endTransmission(false); 

  Wire.requestFrom(MPU6050_ADDRESS, 14, true);

  accelData[0] = Wire.read() << 8 | Wire.read(); // accH and AccL
  accelData[1] = Wire.read() << 8 | Wire.read(); // accH and accL
  accelData[2] = Wire.read() << 8 | Wire.read(); // accH and accL
  int16_t a= Wire.read()<< 8| Wire.read();      // To skip temp sensor
  gyroData[0] = Wire.read() << 8 | Wire.read(); // gyroH and gyroL
  gyroData[1] = Wire.read() << 8 | Wire.read(); // gyroH and gyroL
  gyroData[2] = Wire.read() << 8 | Wire.read(); // gyroH and gyroL
}
