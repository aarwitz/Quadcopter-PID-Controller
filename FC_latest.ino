


#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Servo.h>

#define BNO055_SAMPLERATE_DELAY_MS (1000)
#define ROLL 0
#define PITCH 1
#define YAW 2

int THRUST_MINIMUM = 1000;
int THRUST_MAXIMUM = 1850;

/*
 *      Setup
 */

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);



double thrust = 1350;

/*
 *      Constants
 */

float filter = 0.9;                                 // Complementary filter for pid
float gain_p[3] = {3.5, 3.5, 3.5};
float gain_i[3] = {0, 0, 0};
float gain_d[3] = {0.05, 0.05, 0.05};



/*
 *      Variables
 */

float error_current[3] = {0, 0, 0};                 // Current error
float error_prev[3] = {0, 0, 0};                    // Previous error
float orientation_current[3] = {0.0, 0.0, 0.0};
float orientation_setpoint[3] = {0.0, 0.0, 0.0};

float pid_current[3] = {0, 0, 0};                   // PID weighted (!) sum of proportional, integral and derivitive error
float pid_p[3] = {0, 0, 0};                         // PID proportional error     
float pid_i[3] = {0, 0, 0};                         // PID integral error
float pid_d[3] = {0.1, 0.1, 0.1};                         // PID derivitive error

float dt = 50;                                      // Loop every 50ms
float time;


unsigned long time_current;                                 // Current time
unsigned long time_prev;                                    // Previous time
unsigned long time_elapsed;                                // Elapsed time during the last loop

Servo motor1; // front right
Servo motor2; // front left
Servo motor3; // bacck right
Servo motor4; // back left

bool times_up = false;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  
  // put your setup code here, to run once:
  Serial.begin(115200);
  if (!bno.begin()) {
    Serial.print("no imu detected");
    while (1);
  }
  /* Use external crystal for better accuracy */
  bno.setExtCrystalUse(true);
  /* Attach the motors */
  motor1.attach(13);                                
  motor2.attach(11);
  motor3.attach(9);
  motor4.attach(8);

  send_max();
  delay(5000);
  
  calibrateMotors();
  
  calibrateIMU();

  time_prev = millis();
  delay(100);
}


void loop() {
  /* Calculate elapsed time */
  time_current = millis();
  time_elapsed = (time_current - time_prev) / 1000;
  if (time_elapsed > 15) {
    times_up = true;
  }

  while (times_up) {
    writeZero();
    delay(100);
 }
  
  // Get state (updates error_current)
  get_gyro_measurement();   
 
  // Calculate PID
  calculatePID();
  // Define input to system
  writePID2Motors();
  delay(100);
//  Serial.print("Pitch setpoint: ");
//  Serial.print(orientation_setpoint[PITCH]);
//  Serial.print("Pitch actual: ");
//  Serial.print(orientation_current[PITCH]);
//  Serial.print("error_current[PITCH]: ");
//  Serial.println(error_current[PITCH]);
//  Serial.print("pid_current[PITCH]: ");
//  Serial.println(pid_current[PITCH]);
//  delay(dt);
}


void calculatePID() {
  /* Save previous errors */
  error_prev[PITCH] = error_current[PITCH];
  error_prev[ROLL] = error_current[ROLL];
  error_prev[YAW] = error_current[YAW];
  
  /* Calculate current error */
  error_current[PITCH] = orientation_current[PITCH] - orientation_setpoint[PITCH];
  error_current[ROLL] = orientation_current[ROLL] - orientation_setpoint[ROLL];
  error_current[YAW] = orientation_current[YAW] - orientation_setpoint[YAW];
//  Serial.print("error_current[ROLL]: ");
//  Serial.print(error_current[ROLL]);
//  Serial.print(" error_current[PITCH]: ");
//  Serial.print(error_current[PITCH]);
//  Serial.print(" error_current[YAW]: ");
//  Serial.println(error_current[YAW]);

  /* Calculate weighted proportional error */
  pid_p[PITCH] = gain_p[PITCH] * error_current[PITCH];
  pid_p[ROLL] = gain_p[ROLL] * error_current[ROLL];
  pid_p[YAW] = gain_p[YAW] * error_current[YAW];
//  Serial.print("pid_p[ROLL]: ");
//  Serial.print(pid_p[ROLL]);
//  Serial.print(" pid_p[PITCH]: ");
//  Serial.print(pid_p[PITCH]);
//  Serial.print(" pid_p[YAW]: ");
//  Serial.println(pid_p[YAW]);

   /* Calculated weighted derivitive error */
  float pid_d_new[3];
//  Serial.print("time_elapssed: ");
//  Serial.println(time_elapsed);
//  Serial.print("filter: ");
//  Serial.println(filter);
//   Serial.print("1- filter: ");
//  Serial.println(1 - filter);
//  Serial.print("gain_d[PITCH]: ");
//  Serial.println(gain_d[PITCH]);
//  Serial.print("error_prev[PITCH]: ");
//  Serial.println(error_prev[PITCH]);

  pid_d_new[PITCH] = gain_d[PITCH] * (error_current[PITCH] - error_prev[PITCH]) / float(time_elapsed);
  pid_d_new[ROLL] = gain_d[ROLL] * (error_current[ROLL] - error_prev[ROLL]) / float(time_elapsed);
  pid_d_new[YAW] = gain_d[YAW] * (error_current[YAW] - error_prev[YAW]) / float(time_elapsed);

  
//  Serial.print("pid_d_new[ROLL]: ");
//  Serial.print(pid_d_new[ROLL]);
//  Serial.print(" pid_d_new[PITCH]: ");
//  Serial.print(pid_d_new[PITCH]);
//  Serial.print(" pid_d_new[YAW]: ");
//  Serial.println(pid_d_new[YAW]);
//  
  pid_d[PITCH] = filter * pid_d[PITCH] + (1 - filter) * pid_d_new[PITCH];
  pid_d[ROLL] = filter * pid_d[ROLL] + (1 - filter) * pid_d_new[ROLL];
  pid_d[YAW] = filter * pid_d[YAW] + (1 - filter) * pid_d_new[YAW];
//
//  Serial.print("pid_d[ROLL]: ");
//  Serial.print(pid_d[ROLL]);
//  Serial.print(" pid_d[PITCH]: ");
//  Serial.print(pid_d[PITCH]);
//  Serial.print(" pid_d[YAW]: ");
//  Serial.println(pid_d[YAW]);
  /* Calculate weighted sum of the PID */
//  pid_current[PITCH] = pid_p[PITCH] + pid_i[PITCH] + pid_d[PITCH];
//  pid_current[ROLL] = pid_p[ROLL] + pid_i[ROLL] + pid_d[ROLL];
//  pid_current[YAW] = pid_p[YAW] + pid_i[YAW] + pid_d[YAW];
  pid_current[PITCH] = pid_p[PITCH];
  pid_current[ROLL] = pid_p[ROLL];
  pid_current[YAW] = pid_p[YAW];
  Serial.print("pid_p[PITCH]: ");
  Serial.println(pid_p[PITCH]);
  Serial.print("pid_p[YAW]: ");
  Serial.println(pid_p[YAW]);
  Serial.print("pid_p[ROll]: ");
  Serial.println(pid_p[ROLL]);
  pinMode(LED_BUILTIN, OUTPUT);
}

void writePID2Motors() {
  // Motor mixing algo
  float motor1_cmd = thrust + pid_current[ROLL] + pid_current[PITCH] + pid_current[YAW];
  float motor2_cmd = thrust - pid_current[ROLL] + pid_current[PITCH] - pid_current[YAW];
  float motor3_cmd = thrust + pid_current[ROLL] - pid_current[PITCH] - pid_current[YAW];
  float motor4_cmd = thrust - pid_current[ROLL] - pid_current[PITCH] + pid_current[YAW];
//  Serial.print("thrust: ");
//  Serial.print(thrust);
//  Serial.print(" pid_current[ROLL]: ");
//  Serial.print(pid_current[ROLL]);
//  Serial.print(" pid_current[PITCH]: ");
//  Serial.print(pid_current[PITCH]);
//  Serial.print(" pid_current[YAW]: ");
//  Serial.println(pid_current[YAW]);

  if (motor1_cmd > THRUST_MAXIMUM) {motor1_cmd = THRUST_MAXIMUM;}
  if (motor2_cmd > THRUST_MAXIMUM) {motor1_cmd = THRUST_MAXIMUM;}
  if (motor3_cmd > THRUST_MAXIMUM) {motor1_cmd = THRUST_MAXIMUM;}
  if (motor4_cmd > THRUST_MAXIMUM) {motor1_cmd = THRUST_MAXIMUM;}

  if (motor1_cmd < THRUST_MINIMUM) {motor2_cmd = THRUST_MINIMUM;}
  if (motor2_cmd < THRUST_MINIMUM) {motor2_cmd = THRUST_MINIMUM;}
  if (motor3_cmd < THRUST_MINIMUM) {motor2_cmd = THRUST_MINIMUM;}
  if (motor4_cmd < THRUST_MINIMUM) {motor2_cmd = THRUST_MINIMUM;}
  
  motor1.writeMicroseconds(motor1_cmd);
  motor2.writeMicroseconds(motor2_cmd);
  motor3.writeMicroseconds(motor3_cmd);
  motor4.writeMicroseconds(motor4_cmd);
//
//  Serial.print("motor1_cmd: ");
//  Serial.print(motor1_cmd);
//  Serial.print(" motor2_cmd: ");
//  Serial.print(motor2_cmd);
//  Serial.print(" motor3_cmd: ");
//  Serial.println(motor3_cmd);
}

/**
 * LOW
 * (DELAY 7 SEC)
 * HIGH
 * (DELAY 2 SEC)
 * LOW
 * (DELAY 2 SEC)
 */
void calibrateMotors() {
    delay(2000);
    /*----LOW----*/
    motor1.writeMicroseconds(THRUST_MINIMUM);
    motor2.writeMicroseconds(THRUST_MINIMUM);
    motor3.writeMicroseconds(THRUST_MINIMUM);
    motor4.writeMicroseconds(THRUST_MINIMUM);
    delay(2000);
    /*----HIGH----*/
    motor1.writeMicroseconds(THRUST_MAXIMUM);
    motor2.writeMicroseconds(THRUST_MAXIMUM);
    motor3.writeMicroseconds(THRUST_MAXIMUM);
    motor4.writeMicroseconds(THRUST_MAXIMUM);
    delay(5000);
    /*----LOW----*/
    motor1.writeMicroseconds(thrust);
    motor2.writeMicroseconds(thrust);
    motor3.writeMicroseconds(thrust);
    motor4.writeMicroseconds(thrust);
    delay(10000);
}


void writeZero() {
    /*----LOW----*/
    motor1.writeMicroseconds(THRUST_MINIMUM);
    motor2.writeMicroseconds(THRUST_MINIMUM);
    motor3.writeMicroseconds(THRUST_MINIMUM);
    motor4.writeMicroseconds(THRUST_MINIMUM);
}

void calibrateIMU() {
  bool notCalibrated = true;
  unsigned long start = millis();
  unsigned long now = start;
  while(notCalibrated) {
      /* Print calibration data for sensor. */
      uint8_t sys, gyro, accel, mag = 0;
      bno.getCalibration(&sys, &gyro, &accel, &mag);
//      Serial.print(F("Calibration: "));
//      Serial.print(sys, DEC);
//      Serial.print(F(" "));
//      Serial.print(gyro, DEC);
//      Serial.print(F(" "));
//      Serial.print(accel, DEC);
//      Serial.print(F(" "));
//      Serial.println(mag, DEC);
      
      now = millis();
      if (now - start > 2000) {
        notCalibrated = false;
      }
  }
}

void get_gyro_measurement() {
/* 
 *   Returns {roll, pitch, yaw}
 *  
 *  
 *  Board layout:
       +----------+
       |         *| RST   PITCH  ROLL   YAW
   ADR |*        *| SCL
   INT |*        *| SDA     ^           /->
   PS1 |*        *| GND     |           |
   PS0 |*        *| 3VO     Y    Z-->   \-X
       |         *| VIN
       +----------+
 */
  sensors_event_t event;
  bno.getEvent(&event);
  orientation_current[ROLL] = (float)event.orientation.z;
  orientation_current[PITCH] = (float)event.orientation.y;
  orientation_current[YAW] = (float)event.orientation.x;
  //print_gyro_mesauremnts();
}


void print_gyro_mesauremnts() {
  Serial.print("Roll:");
  Serial.print(orientation_current[ROLL]);
  Serial.print(" Pitch:");
  Serial.print(orientation_current[PITCH]);
  Serial.print(" Yaw:");
  Serial.println(orientation_current[YAW]);
}

void send_max() {
    /*----HIGH----*/
  motor1.writeMicroseconds(THRUST_MAXIMUM);
  motor2.writeMicroseconds(THRUST_MAXIMUM);
  motor3.writeMicroseconds(THRUST_MAXIMUM);
  motor4.writeMicroseconds(THRUST_MAXIMUM);
}




// eval eom (UpdateState) this is where the sensor goes too

// get state
// define wRb using drone state
// drone_world = wRb * drone_body
// simulate drone in world now that you have global
//


// updatestae
// t = t + dt
// x = x + dx * dt
//
