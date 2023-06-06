#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Servo.h>

// instantiate of the gyroscope
Adafruit_BNO055 bno = Adafruit_BNO055(); 

// pins used to connect rc remote to the Arduino
#define RCPinFWD 3
#define RCPinSide 5 


// pins used to connect motor control
const int leftMotorPin = 6;
const int rightMotorPin = 9;

// motor servos 
Servo L;
Servo R;

// constants to calculate balancing based on gyroscope values
#define angle_multiplier 0.4
#define acceleration_multiplier -0.065

// constant for controling amount of motion foward and steering
#define forward_motion_multiplier 0.9
#define steering_multiplier 0.9

// neutral motor speed
const int neutral_speed = 1500;

// setup which assign the pins correctly and starts up the motor
void setup() {
  L.attach(leftMotorPin);
  R.attach(rightMotorPin);
  Serial.begin(115200); // Serial to PC

  // initialize gyroscope
  bno.begin();

  // set motor control pins as outputs
  pinMode(leftMotorPin, OUTPUT);
  pinMode(rightMotorPin, OUTPUT);

  // set rc remote control pins as input
  pinMode(RCPinFWD, INPUT);
  pinMode(RCPinSide, INPUT);

  // Set initial motor power to neutral
  L.writeMicroseconds(neutral_speed);
  R.writeMicroseconds(neutral_speed);

  // gyroscope
  if (!bno.begin()) {
    Serial.print("No BNO055 detected");
    while (1); // halt for safety
  }
  delay(1000);
  bno.setExtCrystalUse(true);
}

// constant looop for motion controller
void loop() {
  motionController(); 
}

void motionController() {
  Serial.println("-----------------------------");
  
  // sample gyroscope values
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

  // calculate how much motors need to change to balance the robots based on gyroscope
  float balance_value = ((euler.z()+9.31) * angle_multiplier + gyro.x() * acceleration_multiplier)*20;
  // print angle and accelaration from gyroscope
  Serial.print("angle:");
  Serial.println(euler.z());
  Serial.print("accelaration:");
  Serial.println(gyro.x());

  // rc remote values read the pulse width from rc pins
  int rc_pulse1 = pulseIn(RCPinFWD, HIGH);  
  int rc_pulse2 = pulseIn(RCPinSide, HIGH); 
  
  // controller input from rc remote 
  // position is the forward motion or backwards motions
  float forward_value = forward_motion_multiplier * (rc_pulse1 - neutral_speed);
  // steering amount for for the wheels
  float steering_value = steering_multiplier * (rc_pulse2 - neutral_speed);  

  // change motor speeds according to all controllers
  float controllerOutput_right = balance_value + forward_value + steering_value;
  float controllerOutput_left  = balance_value + forward_value - steering_value;
  
  int leftPower = static_cast<int>(neutral_speed - controllerOutput_left);
  int rightPower = static_cast<int>(neutral_speed - controllerOutput_right);
  
  // Constrain the motor powers within the allowable range
  leftPower = constrain(leftPower, 1000, 2000);
  rightPower = constrain(rightPower, 1000, 2000);
  // print motor power
  Serial.print("left power");
  Serial.println(leftPower);
  Serial.print("right power");
  Serial.println(rightPower);
  
  // set motor powers
  L.writeMicroseconds(leftPower);
  R.writeMicroseconds(rightPower);
}
