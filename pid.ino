
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#define BNO055_SAMPLERATE_DELAY_MS (1000)

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

const float KP = 0.5;  // Proportional gain
const float KI = 0.2;  // Integral gain
const float KD = 10.0;  // Derivative gain
const float ACCEL_NOISE_STDDEV = 0.1;  // m/s^2
const float GYRO_NOISE_STDDEV = 0.01;  // rad/s

//float setpoint[3] = {0.0, 0.0, 0.0};
//float input[3] = {0.0, 0.0, 0.0};
//float output[3] = {0.0, 0.0, 0.0};
float setpoint = 0;   // rad
float dt = 50;  // Loop every 50ms
float time;
float actual, error, u;



void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  if (!bno.begin()) {
    Serial.print("no imu detected");
    while (1);
  }
  time = millis();

  delay(1000);

  /* Use external crystal for better accuracy */
  bno.setExtCrystalUse(true);
}


float get_measurement() {
  sensors_event_t event;
  bno.getEvent(&event);
  actual = (float)event.orientation.z;
  /* Also send calibration data for each sensor. */
  // uint8_t sys, gyro, accel, mag = 0;
  //bno.getCalibration(&sys, &gyro, &accel, &mag);


  return actual;
}

float controller() {
  // Get state
  actual = get_measurement();
  // Calculate error
  error = setpoint - actual;
  // Define input to system
  u = KP * error;

  Serial.print("Setpoint: ");
  Serial.print(setpoint);
  Serial.print("  Actual: ");
  Serial.print(actual);
  Serial.print("  u: ");
  Serial.println(u);
  return u;
}

void loop() {
  // Get state
  u = controller();
  delay(BNO055_SAMPLERATE_DELAY_MS);
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
