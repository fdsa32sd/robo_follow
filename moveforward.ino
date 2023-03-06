#include <QTRSensors.h>
// Right Low
#define motor1low 10
// Right High
#define motor1high 11
// Left Low
#define motor2low 12
// Left High
#define motor2high 13
// high is forward and low is reverse, 1 is left and 2 is right

#define blackirsense 1000
#define loop_interval 250
#define Kp 1
#define Ki 1
#define Kd 1

const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];
const uint8_t irpins[SensorCount] = {2, 3, 4, 5, 6, 7, 8, 9};

double prev_left_error = 0;
double prev_right_error = 0;
double left_integral = 0;
double right_integral = 0;

QTRSensors qtr;

void setup() {
  pinMode(motor1low, OUTPUT);
  pinMode(motor2low, OUTPUT);
  pinMode(motor1high, OUTPUT);
  pinMode(motor2high, OUTPUT);

  // configure the sensors
  qtr.setTypeRC();
  qtr.setSensorPins(irpins, SensorCount);
  qtr.setEmitterPin(2);

  delay(500);

  // 2.5 ms RC read timeout (default) * 10 reads per calibrate() call
  // = ~25 ms per calibrate() call.
  // Call calibrate() 400 times to make calibration take about 10 seconds.
  for (uint16_t i = 0; i < 400; i++)
  {
    qtr.calibrate();
  }

  // print the calibration minimum values measured when emitters were on
  Serial.begin(9600);
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(' ');
  }
  Serial.println();

  // print the calibration maximum values measured when emitters were on
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.maximum[i]);
    Serial.print(' ');
  }
  Serial.println();
  forward();
  delay(1000);
}


double checkL() {
  int total = 0; 
  for (uint8_t i = 0; i < 3; i++)
  {
    total += sensorValues[i]; 
  }  

  double avg = total / 3;
  return avg; 
}

double checkR() {
  int total = 0; 
  for (uint8_t i = 5; i < 8; i++)
  {
    total += sensorValues[i]; 
  }  

  double avg = total / 3; 
  return avg; 
}

void forward() {
  digitalWrite(motor1low, LOW);
  digitalWrite(motor1high, HIGH);
  digitalWrite(motor2low, LOW);
  digitalWrite(motor2high, HIGH);
}

void stop() {
  digitalWrite(motor1low, LOW);
  digitalWrite(motor1high, LOW);
  digitalWrite(motor2low, LOW);
  digitalWrite(motor2high, LOW);
}

void loop() {
  
  // read calibrated sensor values and obtain a measure of the line position
  // from 0 to 5000 (for a white line, use readLineWhite() instead)
  uint16_t position = qtr.readLineWhite(sensorValues);

  // print the sensor values as numbers from 0 to 1000, where 0 means maximum
  // reflectance and 1000 means minimum reflectance, followed by the line
  // position
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
  double left_ir = checkL();
  double right_ir = checkR();
  Serial.println(left_ir); 
  Serial.println(right_ir);

  if (left_ir == 0 && right_ir == 0) {
    stop();
    return;
  }
  // pid control
  double left_error = blackirsense - left_ir;
  double right_error = blackirsense - right_ir;
  double left_p = Kp * left_error;
  double right_p = Kp * right_error;
  left_integral = left_integral + left_error * loop_interval;
  double left_i = Ki * left_integral;
  right_integral = right_integral + right_error * loop_interval;
  double right_i = Ki * right_integral;
  double left_d = Kd * (left_error - prev_left_error)/loop_interval;
  double right_d = Kd * (right_error - prev_right_error)/loop_interval;
  double output_left = left_p + left_i + left_d;
  double output_right = right_p + right_i + right_d;
  

  delay(loop_interval);
}
