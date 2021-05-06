/***************************************************
  Bicycle Power i(ndirect)Meter
  Sum of Forces

  This sketch combines 4 sensors into one!
    - Adafruit LIS3DH Accelerometer
    - Adafruit L3GD20 Gyroscope
    - Adafruit BMP280 Barometer
    - Modern Device Wind Sensor Rev. C

  The 3 Adafruit sensors communicate via I2C. The Modern Device Wind Sensor is an analog in.

  The sum of forces is output as an analog out!

  The sketch is based off of and uses published Adafruit and Modern Device libraries/code. Thanks!

  Written by Alex Gui
  For UW EE PMP EE542 Quarter Project, Summer 2016
 ****************************************************/

#include <Wire.h>
#include <Adafruit_LIS3DH.h>
#include <Adafruit_L3GD20.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>

#define analogPinOut 10 // This is the Arduino pin to output Fsum!

typedef struct {
  double q; // process noise covariance
  double r; // measurement noise covariance
  double p; // estimation erro covariance
  double k; // kalman gain
  double x; // value
} kalman_state;

kalman_state my_kalman;
kalman_state* my_kalman_ptr = &my_kalman;

// User defined constants
float Mr = 66; // Rider mass, kg
float Mb = 9; // Bicycle mass, kg
float M; // Total mass of rider + bicycle, kg

// Other constants
float g = 9.81; // Gravitational constant, m/s^2
float Cd = 1.0; // Coefficient of drag for cyclist; Tops = 1.15, Hoods = 1.0, Drops = 0.88, Aero Bars = 0.70; Reference: http://www.cyclingpowerlab.com/CyclingAerodynamics.aspx
float area = 0.40; // Frontal area of cyclist, m^2; Tops = 0.632, Hoods = 0.40, Drops = 0.32; Reference: same as above
float Crr = 0.0042; // Coefficient of rolling resistance; bicycle tire on smooth pavement at 100 psi; Reference: https://silca.cc/blogs/journal/part-4b-rolling-resistance-and-impedance
float Rs_dryAir = 287.058; // Specific gas constant for dry air, J/(kg*K)
float pi = 3.14159;

// Some variables we'll use later
float theta = 0.0; // Y axis pitch, degrees
float thetaA = 0.0; // Y axis pitch, degrees, calculated from accelerometer data
float rho; // Air density, kg/m^3
float Ax; // X axis acceleration
float Ay;
float Az;
//float AxP = 0.0; // Previous X axis acceleration
//float AyP = 0.0;
//float AzP = 9.81;
float Gx;
float Gy; // Y axis rotation
float Gz;
float xAccel = 0.0; // X axis acceleration due to translation

// Forces, all in Newtons
float Fd; // Drag force
float Fa; // X-Acceleration force
float Fg; // Gravitational force
float Ff; // Friction force
float Fsum; // Sum of forces
float FsumOut; // Converted on scale of 0-255 for PWM (analog) output
float Fmax = 200; // N, A reasonable maximum force value?? *Roughly* based off of acceleration to 20 mph in 3 seconds w/ 70 kg

// Calibration values for sensors
float accelXOffset = 0;
float accelYOffset = 0;
float accelZOffset = 0;
float gyroXOffset = 0;
float gyroYOffset = 0;
float gyroZOffset = 0;

int samplingPeriod = 100; // ms
int nCalibSamples = 1000; // 100 samples with 1 ms delay is ~0.1 seconds!
unsigned long lastMillisGyro;
unsigned long dt;
int gyroMult = 1;// multiplier for y axis gyroscope direction
int accMult = 1;

// I2C
Adafruit_LIS3DH lis = Adafruit_LIS3DH();
Adafruit_L3GD20 gyro;
Adafruit_BMP280 bme;

// Define stuff for the anemometer
#define analogPinForRV    0   // These are the analog pins on the Arduino we're using
#define analogPinForTMP   1
// To calibrate the sensor, put a glass over it (don't let the sensor touch the desk surface!).
// Adjust the following value until the thingy reads zero.
const float zeroWindAdjustment = 0.2;//0.1; // Negative numbers yield smaller wind speeds and vice versa.
// Other stuff use for the anemometer
int TMP_Therm_ADunits; // Temp termistor value from wind sensor
float RV_Wind_ADunits; // RV output from wind sensor
float RV_Wind_Volts;
unsigned long lastMillis;
int TempCtimes100;
float zeroWind_ADunits;
float zeroWind_volts;
float WindSpeed_MPH;
float WindSpeed_mps; // WindSpeed in meters per second

void setup() {
  // This is the setup code! It runs once!
  Serial.begin(9600);
  Serial.println("bikePower");

  // Check for LIS3DH
  if (!lis.begin(0x18)) {   // change this to 0x19 for alternative i2c address
    Serial.println("Could not find LIS3DH!");
    while (1);
  }
  Serial.println("LIS3DH found!");

  // Check for L3GD20H
  if (!gyro.begin(gyro.L3DS20_RANGE_250DPS))
  //if (!gyro.begin(gyro.L3DS20_RANGE_500DPS))
  //if (!gyro.begin(gyro.L3DS20_RANGE_2000DPS))
  {
    Serial.println("Could not find L3GD20!");
    while (1);
  }
  Serial.println("L3GD20 found!");

  // Check for BMP280
  if (!bme.begin()) {
    Serial.println("Could not find BMP280!");
    while (1);
  }
  Serial.println("BMP280 found!");

  // We can set the range of the accelerometer here
  lis.setRange(LIS3DH_RANGE_2_G);   // 2, 4, 8 or 16 G!
  Serial.print("LIS3DH Range = ");
  Serial.print(2 << lis.getRange());
  Serial.println("G");

  Serial.println("!Initialization complete - All Sensors Initialized!");
  Serial.println();

  // Loop 100 times and read the accel and gyro to figure out the X offset
  sensors_event_t acc_event;
  lis.getEvent(&acc_event);
  for (int x = 0; x < nCalibSamples; x++)
  {
    sensors_event_t acc_event;
    lis.getEvent(&acc_event);
    accelXOffset += accMult*acc_event.acceleration.y; // Note: DeviceX = -SensorY, DeviceY = SensorX, DeviceZ = SensorZ
    accelYOffset += acc_event.acceleration.x;
    accelZOffset += acc_event.acceleration.z;

    gyro.read();
    gyroXOffset += gyro.data.y; // Note: DeviceX = SensorY, DeviceY = -SensorX, DeviceZ = SensorZ
    gyroYOffset += gyroMult*gyro.data.x;
    gyroZOffset += gyro.data.z;

    //delay(0.1); // Delay 1 ms, so this 100 loops is equal to ~0.1 second!
  }
  accelXOffset /= nCalibSamples; accelYOffset /= nCalibSamples; accelZOffset /= nCalibSamples; accelZOffset -= g;
  gyroXOffset /= nCalibSamples; gyroYOffset /= nCalibSamples; gyroZOffset /= nCalibSamples;
  Serial.println();
  Serial.print("Accel X offset: "); Serial.println(accelXOffset); Serial.print(" ");
  Serial.print("Accel Y offset: "); Serial.println(accelYOffset); Serial.print(" ");
  Serial.print("Accel Z offset: "); Serial.println(accelZOffset); Serial.print(" ");
  Serial.print("Gyro X offset: "); Serial.println(gyroXOffset); Serial.print(" ");
  Serial.print("Gyro Y offset: "); Serial.println(gyroYOffset); Serial.print(" ");
  Serial.print("Gyro Z offset: "); Serial.println(gyroZOffset); Serial.print(" ");
  Serial.println();

  // Slow but smooth
  double q = 0.0625;
  double p = 1.383;
  double r = 32;
  // Fast & good but a bit too much noise?
//  double q = 0.125;
//  double p = 1.06;
//  double r = 10;
  // Set up Kalman filter
  my_kalman = kalman_init(q,r,p,0);
  
}

// Simple first order complementary filter for gyroscope y axis pitch filtering
// This should reduce gyro drift a bit...
float Complementary(float oldAngle, float newAngle, float newRate, int loopTime)
{
  float a = 0.99;
  //float tau = 0.075;
  float dtC = float(loopTime)/1000.0; // Convert loopTime from ms to s

  //a = tau/(tau + dtC);
  float thetaC = a*(oldAngle + newRate*dtC) + (1 - a)*newAngle;

//  Serial.print("looptime (s): "); Serial.println(dtC);
//  Serial.print("thetaC: "); Serial.println(thetaC);
//  Serial.print("newRate: "); Serial.println(newRate);
//  Serial.print("oldAngle: "); Serial.println(oldAngle);
//  Serial.print("newAngle: "); Serial.println(newAngle);

  return thetaC;
}

// Kalman filter intializer
kalman_state kalman_init(double q, double r, double p, double initial_value)
{
  kalman_state state;
  state.q = q;
  state.r = r;
  state.p = p;
  state.x = initial_value;

  return state;
}

// Kalman filter update function
void kalman_update(kalman_state* state, double measurement)
{
  // update prediction
  state->p = state->p + state->q;

  // update measurement
  state->k = state->p / (state->p + state->r);
  state->x = state->x + state->k * (measurement - state->x);
  state->p = (1 - state->k)*state->p;
}

// analogWrite tester function
void test_analogWrite(void)
{
  analogWrite(analogPinOut,255*4/4); // Pin Value (0-255) = 255 * (AnalogVolts / 5);
  delay(5000);
  analogWrite(analogPinOut,255*3/4); // Pin Value (0-255) = 255 * (AnalogVolts / 5);
  delay(5000);
  analogWrite(analogPinOut,255*2/4); // Pin Value (0-255) = 255 * (AnalogVolts / 5);
  delay(5000);
  analogWrite(analogPinOut,255*1/4); // Pin Value (0-255) = 255 * (AnalogVolts / 5);
  delay(5000);
  analogWrite(analogPinOut,255*0); // Pin Value (0-255) = 255 * (AnalogVolts / 5);
  delay(5000);
}

void loop() {
  // This is the main loop! It runs forever!

  // Use a sensor object to read the accelerometer
  sensors_event_t acc_event;
  lis.getEvent(&acc_event);
  /* Display the results (acceleration is measured in m/s^2) */
//  Serial.print("aX: "); Serial.print(accMult * acc_event.acceleration.y); Serial.print(" "); // Note: DeviceX = -SensorY
//  Serial.print("aY: "); Serial.print(acc_event.acceleration.x); Serial.print(" ");
//  Serial.print("aZ: "); Serial.print(acc_event.acceleration.z); Serial.print(" ");
//  Serial.println(" m/s^2 ");
//  Serial.println();
  
  // Read the gyro
//  gyro.read();
//  Serial.print("gX: "); Serial.print((int)gyro.data.y); Serial.print(" ");
//  Serial.print("gY: "); Serial.print((int)(gyroMult * gyro.data.x)); Serial.print(" "); // Note: DeviceY = -SensorX
//  Serial.print("gZ: "); Serial.print((int)gyro.data.z); Serial.print(" ");
//  Serial.print("gX: "); Serial.print((int)gyro.data.y - gyroXOffset); Serial.print(" ");
//  Serial.print("gY: "); Serial.print((int)(gyroMult * gyro.data.x - gyroYOffset)); Serial.print(" "); // Note: DeviceY = -SensorX
//  Serial.print("gZ: "); Serial.print((int)gyro.data.z - gyroZOffset); Serial.print(" ");
//  Serial.println(" dps "); Serial.println();

  // Read the barometer
//  Serial.print("Temperature = "); Serial.print(bme.readTemperature()); Serial.println(" *C");
//  Serial.print("Pressure = "); Serial.print(bme.readPressure()); Serial.println(" Pa");
//  Serial.print("Approx altitude = "); Serial.print(bme.readAltitude(1013.25)); Serial.println(" m"); // Adjust the value to your local forecast!!
//  Serial.println();
  // Read the anemometer
  if (millis() - lastMillis > 200) {     // Read every 200 ms - printing slows this down further
    TMP_Therm_ADunits = analogRead(analogPinForTMP);
    RV_Wind_ADunits = analogRead(analogPinForRV);
    RV_Wind_Volts = (RV_Wind_ADunits *  0.0048828125);

    // These are all derived from regressions from raw data as such they depend on a lot of experimental factors...
    // such as accuracy of temp sensors, and voltage at the actual wind sensor, (wire losses) which were unaccouted for.
    TempCtimes100 = (0.005 * ((float)TMP_Therm_ADunits * (float)TMP_Therm_ADunits)) - (16.862 * (float)TMP_Therm_ADunits) + 9075.4;
    zeroWind_ADunits = -0.0006 * ((float)TMP_Therm_ADunits * (float)TMP_Therm_ADunits) + 1.0727 * (float)TMP_Therm_ADunits + 47.172; //  13.0C  553  482.39
    zeroWind_volts = (zeroWind_ADunits * 0.0048828125) - zeroWindAdjustment;

    // This from a regression from data in the form of
    // Vraw = V0 + b * WindSpeed ^ c
    // V0 is zero wind at a particular temperature
    // The constants b and c were determined by some Excel wrangling with the solver.
    WindSpeed_MPH =  pow(((RV_Wind_Volts - zeroWind_volts) / .2300), 2.7265);
    WindSpeed_mps = WindSpeed_MPH * 0.44704; // Wind speed in meters per second!

//    Serial.print("TMP volts: "); Serial.print(TMP_Therm_ADunits * 0.0048828125); Serial.println(" ");
//    Serial.print("RV volts: "); Serial.print((float)RV_Wind_Volts); Serial.println(" ");
//    Serial.print("TempC*100 "); Serial.print(TempCtimes100 ); Serial.println(" ");
//    Serial.print("ZeroWind volts: "); Serial.print(zeroWind_volts); Serial.println(" ");
//    Serial.println();
//    Serial.print("WindSpeed MPH: "); Serial.println((float)WindSpeed_MPH); Serial.println();
    lastMillis = millis();
  }

  // Do the force calculation
  M = Mr + Mb; // Total mass of rider + bike, kg
  rho = bme.readPressure() / (Rs_dryAir*(bme.readTemperature() + 273.15)); // kg/m^3, Pressure in Pa, Temperature in *C converted to K

  Ax = accMult*acc_event.acceleration.y - accelXOffset;
  Ay = acc_event.acceleration.x - accelYOffset;
  Az = acc_event.acceleration.z - accelZOffset;
//  Serial.print("aX: "); Serial.print(Ax); Serial.print(" ");
//  Serial.print("aY: "); Serial.print(Ay); Serial.print(" ");
//  Serial.print("aZ: "); Serial.print(Az); Serial.print(" ");

//  double temp = (AxP*Ax + AyP*Ay + AzP*Az)/(sqrt(sq(AxP)+sq(AyP)+sq(AzP))*sqrt(sq(Ax)+sq(Ay)+sq(Az)));
//  if (temp>=1)
//  {
//    temp = 1;
//  }
//  if (temp<=-1)
//  {
//    temp = -1;
//  }
//  theta += (acos(temp)*180/pi);
  theta = -atan(Ax/sqrt(sq(Ay)+sq(Az)))*180/pi;
//  Serial.print("Theta Raw: "); Serial.println((float)theta);
  kalman_update(my_kalman_ptr,theta);
  theta = my_kalman_ptr->x;
//  theta = 0;
//  thetaA = atan(Ay/sqrt(pow(Ax,2) + pow(Az,2)))*180/pi; // Pitch about the y axis calculated with accelerometer data, in degrees
//  Serial.print("Theta Accel: "); Serial.println((float)theta);
// Read gyro, get angular rate about y axis
//  gyro.read();
//  Gy = (gyroMult*gyro.data.x - gyroYOffset)*180/pi; // Angular rate about y axis in dps
//  Serial.print("Theta Rate DPS: "); Serial.println((float)Gy);
//  dt = millis() - lastMillisGyro;
//  //Serial.print("gyro dt "); Serial.println((unsigned long)dt);
//  theta = Complementary(theta,thetaA,Gy,dt);
//  lastMillisGyro = millis();
  if (theta>=90)
  {
    theta = 90;
  }
  if (theta<=-90)
  {
    theta = -90;
  }
  
//  // Save the values from this round for the next iteration
//  AxP = Ax;
//  AyP = Ay;
//  AzP = Az;

  // Extract the x-acceleration from the rider by taking the magnitude of the accelerations and substracting a 1 g force.
  // This obviously assumes that any additional acceleration beyond 1g is due to the rider. Is it right? It's a big assumption.
  xAccel = sqrt(sq(Ax) + sq(Ay) + sq(Az)) - g;
  
  Fd = Cd*area*rho*sq(WindSpeed_mps)/2;
  Fa = 0;//M*xAccel;
  Fg = M*g*sin(theta*pi/180); // Input angle in radians
  Ff = Crr*M*g*cos(theta*pi/180); // Input angle in radians
  Fsum = Fd + Fa + Fg + Ff;
  //Fsum = Fd + Fa + Ff; // Fg already accounted for in Fa since Ax includes gravity
  if (Fsum>Fmax)
  {
    Fsum = Fmax;
  }
  if (Fsum<0)
  {
    Fsum = 0;
  }
  
  FsumOut = (Fsum / Fmax) * 255; // Convert to 0 to 255 scale for PWM output
  
//  Serial.print("Ax m/s^2: "); Serial.println((float)xAccel);
//  Serial.print("Theta Deg: "); Serial.println((float)theta);
//  Serial.print("WindSpeed MPH: "); Serial.println((float)WindSpeed_MPH);
//  Serial.print("Air Density kg/m^3: "); Serial.println((float)rho);
//  Serial.print("Force Sum Newtons: "); Serial.println((float)Fsum);
//  Serial.print("Force Sum PWM: "); Serial.println((float)(FsumOut));
//  Serial.println();// Serial.println();

//  test_analogWrite;
  analogWrite(analogPinOut,FsumOut);
  
  delay(samplingPeriod);
}
