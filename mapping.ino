#include <Ultrasonic.h>
#include <Wire.h>
#include <SPI.h>
#include <SparkFunLSM9DS1.h>

//variables for all sensors
int row = 0; //y-axis from forward sensor, value either 1 or 2
int col = 0; //x-axis from side sensor, value either 1 or 2
long height = 0; //value returned by bottom sensor
int quad = 0;
int color;
char a = 'E';

//ultrasonic sensor variables
int forwardtrigger = 3;
int forwardecho = 4;
int sidetrigger = 5;
int sideecho = 6;
int bottomtrigger = 7;
int bottomecho = 8; 

//imu variables
float roll;
float pitch;
float heading;
float AX;
float AY;
float AZ;
float MX;
float MY;
float MZ;
#define LSM9DS1_M  0x1E
#define LSM9DS1_AG  0x6B
#define PRINT_CALCULATED // This line is active - the more-useful calculated values will print - see below
// #define PRINT_RAW // This line is not active (commented out) 
#define PRINT_SPEED 250
#define DECLINATION -12 // Irvine, CA declination 

Ultrasonic front(forwardtrigger, forwardecho);
Ultrasonic side(sidetrigger, sideecho);
Ultrasonic bottom(bottomtrigger, bottomecho);
LSM9DS1 imu;

void setup() {
  // put your setup code here, to run once:
Serial.begin(9600);
 imu.settings.device.commInterface = IMU_MODE_I2C;
  imu.settings.device.mAddress = LSM9DS1_M;
  imu.settings.device.agAddress = LSM9DS1_AG;
  if (!imu.begin()) // This line means "If the IMU does NOT(!) begin, print the following message..."
  {
    Serial.println("Failed to communicate with LSM9DS1.");
    Serial.println("Double-check wiring.");
    Serial.println("Default settings in this sketch will " \
                  "work for an out of the box LSM9DS1 " \
                  "Breakout, but may need to be modified " \
                  "if the board jumpers are.");
    while (1); // wait forever 
  }

/*
pinMode(forwardtrigger, OUTPUT);
pinMode(forwardecho, INPUT);
pinMode(sidetrigger, OUTPUT);
pinMode(sideecho, INPUT);
pinMode(bottomtrigger, OUTPUT);
pinMode(bottomecho, INPUT);
*/
}

int getRow(long distance)  //returns 1 or 2 depending on the quadrant
{

  if (distance <= 152.4)
  {
    return 1;
  }
  else if (distance <= 304.8)
  {
    return 2;
  }
  else
  {
    return 0;
  }
}

int getCol(long distance)  //returns 1 or 2 depending on the quadrant
{
  if (distance <= 152.4)
  {
    return 1;
  }
  else if (distance <= 304.8)
  {
    return 2;
  }
  else
  {
    return 0;
  }
}

int getColor(long distance)
{
  if (distance <= 243.84 && distance >= 198.12)
  {
    return 1;
  }
  else if (distance <= 198.12 && distance >= 152.4)
  {
    return 2;
  }
  else if (distance <= 152.4 && distance >= 106.68)
  {
    return 3;
  }
  else if (distance <= 106.68 && distance >= 60.96)
  {
    return 4;
  }
  else
  {
    return 0;
  }
}

void loop() {
    imu.readGyro();
  Serial.print("G: ");
  #ifdef PRINT_CALCULATED // The values calcuated are in units [degrees / second]
    Serial.print(imu.calcGyro(imu.gx), 2);
    Serial.print(", ");
    Serial.print(imu.calcGyro(imu.gy), 2);
    Serial.print(", ");
    Serial.print(imu.calcGyro(imu.gz), 2);
    Serial.println(" deg/s");
  #elif defined PRINT_RAW
    Serial.print(imu.gx);
    Serial.print(", ");
    Serial.print(imu.gy);
    Serial.print(", ");
    Serial.println(imu.gz);
  #endif

  //print accel values
  imu.readAccel();
    Serial.print("A: ");
  #ifdef PRINT_CALCULATED // The values calculated are in units [g's] where 1g = 9.81 m/s^2
    Serial.print(imu.calcAccel(imu.ax), 2);
    Serial.print(", ");
    Serial.print(imu.calcAccel(imu.ay), 2);
    Serial.print(", ");
    Serial.print(imu.calcAccel(imu.az), 2);
    Serial.println(" g");
  #elif defined PRINT_RAW 
    Serial.print(imu.ax);
    Serial.print(", ");
    Serial.print(imu.ay);
    Serial.print(", ");
    Serial.println(imu.az);
  #endif

  //print mag values
  imu.readMag();
    Serial.print("M: ");
  #ifdef PRINT_CALCULATED // The values calcuated are in units [gauss]
    Serial.print(imu.calcMag(imu.mx), 2);
    Serial.print(", ");
    Serial.print(imu.calcMag(imu.my), 2);
    Serial.print(", ");
    Serial.print(imu.calcMag(imu.mz), 2);
    Serial.println(" gauss");
  #elif defined PRINT_RAW
    Serial.print(imu.mx);
    Serial.print(", ");
    Serial.print(imu.my);
    Serial.print(", ");
    Serial.println(imu.mz);
  #endif

  //calculate attitude values
  AX = imu.ax;
  AY = imu.ay;
  AZ = imu.az;
  MX = imu.mx;
  MY = imu.my;
  MZ = imu.mz;
  
  roll = atan2(AY, AZ); 
  roll *= 180.0 / PI; // Angle of roll [degrees]
  
  pitch = atan2(-AX, sqrt(AY * AY + AZ * AZ));
  pitch *= 180.0 / PI; // Angle of pitch [degrees]

  heading = atan2(MY, MX);
  
  heading -= DECLINATION * PI / 180;
  heading *= 180.0 / PI; // Angle of heading [degrees] where 0 deg = 360 deg = North
  
  Serial.print(roll,2);
  Serial.print(", ");
  Serial.print(pitch,2);
  Serial.print(", ");
  Serial.println(heading,2);
  Serial.println(" ");
  delay(500);

  row = getRow(front.Ranging(CM));
  col = getCol(side.Ranging(CM));
 height = bottom.Ranging(CM); 
 color = getColor(height);
if (roll <= 5 && roll >= -5 && pitch <= 5 && pitch >= -5)
{
  if (row > 0 && col > 0)  // if in a quadrant
  {
    if (row == 1)
    {
      if (col == 1)
      {
        //send to panal that quad is in quadrant 2
        quad = 2;
       // color = 1;
      }
      if (col == 2)
      {
        // send to panal that quad is in quadrant 1
        quad = 1;
       // color = 4;
      }
    }

    if (row == 2)
    {
      if (col == 1)
      {
        //send to panal that quad is in quadrant 3
        quad = 3;
       // color = 4;
      }
      if (col == 2)
      {
        // send to panal that quad is in quadrant 4
        quad = 4;
        //color = 1;
      }
    }
  //  Serial.println("Row = " + row);
  //  Serial.println("Col = " + col);
  Serial.println(bottom.Ranging(CM));
    Serial.println(color);
Serial.println(quad);
    
  }
  // put your main code here, to run repeatedly:
  //quad = 2;
 // color = 2;
  Serial.write(a);

  Serial.write(quad/256);
  Serial.write(quad%256);

  Serial.write(color/256);
  Serial.write(color%256);

  delay(500);
}
}
