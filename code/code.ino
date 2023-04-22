
#include <string>
#include <Arduino_LSM6DS3.h>
#include <array>
#include <numeric> 

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

//Display things:
constexpr size_t SCREEN_WIDTH {128}; // OLED display width, in pixels
constexpr size_t SCREEN_HEIGHT {32}; // OLED display height, in pixels

constexpr int OLED_RESET     {-1}; // Reset pin # (or -1 if sharing Arduino reset pin)
constexpr size_t SCREEN_ADDRESS {0x3C}; ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

std::string s; //Something needed for the accelerometer/gyroscope library to do sampling! 

//Ikke det bedst med defines, men kun 2 så syntes ikke det er så slemt
//Anvendes til at definer trigPin og echoPin for ultralydssensoren. 
const int echoPin {2};       // Connects to the echo pin on the distance sensor.
const int trigPin {3};       // Connects to the trigger pin on the distance sensor.

size_t samplingTime {77}; // Time between sample should be taken in ms. 

//Reading data from the IMU
template <size_t N>
class IMUReader
{
  public:
    IMUReader() = default;


    struct AccGyromeas   //Laver "En datatype" til alle ens målingerne
    {
          float AccX { 0 };
          float AccY { 0 };
          float AccZ { 0 };
          float velTheta { 0 };
          float velPhi { 0 };
          float velPsi { 0 };
    };

    std::array <AccGyromeas, N>Measurements {}; //Laver et array/container til ens målingerne, 
    //N beskriver størrelse af arrayet og er lavet som input når klassen initalerizes

    void GetIMUMeasurement() //Laver en måling og tilføje dem til et array
    {
        float AccX { 0 };
        float AccY { 0 };
        float AccZ { 0 };
        float velTheta { 0 };
        float velPhi { 0 };
        float velPsi { 0 };
        //Getting measurements from the accelerometer and gyropscope
        if (IMU.accelerationAvailable()) 
        {
          IMU.readAcceleration(AccX, AccY, AccZ);
        }
        if (IMU.gyroscopeAvailable()) 
        {
          IMU.readGyroscope(velTheta, velPhi, velPsi);
        }
      //Tilføjer målingerne til et array,således målingerne er pænt gemt 
        
       Measurements.at(index).AccX = AccX;
       Measurements.at(index).AccY = AccY;
       Measurements.at(index).AccZ = AccZ;
       Measurements.at(index).velTheta = velTheta;
       Measurements.at(index).velPhi = velPhi;
       Measurements.at(index).velPsi = velPsi;

       index=index+1; //increment index by 1
       if(index==N)
       {
         index=0; 
       }
       
    }

    
private:
size_t index {0};
};

IMUReader<1> IMURead;  //Initalizers ens klasse til måling af data fra 
//AccGyroMeasurement  ;

class DistanceMeasurement
{
  public:
  
  // RETURNS THE DISTANCE MEASURED BY THE HC-SR04 DISTANCE SENSOR.
  float getDistance()
  {
    // Send out an ultrasonic pulse that's 10 us long.
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    // Use the pulsein command to see how long it takes for the pulse to bounce back to the sensor.
    float echoTime = pulseIn(echoPin, HIGH);       

    // Calculate the distance of the object that reflected the pulse (half the bounce time multiplied by the speed of sound).
    float calculatedDistance = ((echoTime / 148.0f)*2.54)/100; //Regner afstand til et objekt i meter! 

    // Send back the distance that was calculated.
    return(calculatedDistance);              
}

  private:
};

DistanceMeasurement DistMeas; 

//The next class gives the possiablity of changing the sample rate for the IMU!, here it is chosen to change the sample rate to 13 Hz
/* This "trick" is called inheritance. We make a new class that inherits everything from the old class.
   We can then add additional functionality to the new class. In this case, we add four functions to
   add the ability to change the sampling rate for the gyro and accelerometer.
*/
class IMUExtended : public LSM6DS3Class
{
public:
  IMUExtended(TwoWire& wire, uint8_t slaveAddress)
  : LSM6DS3Class{wire, slaveAddress}
  {

  }
  void SetAccGyroRate13Hz()
  {
    writeRegister(0x10, 0b00011000); // See the LSM6DS3 documentation for what to write to these registers.
    writeRegister(0x11, 0b00011100);
  }
  void SetAccGyroRate26Hz()
  {
    writeRegister(0x10, 0b00101000);
    writeRegister(0x11, 0b00101100);
  }
  void SetAccGyroRate52Hz()
  {
    writeRegister(0x10, 0b00111000);
    writeRegister(0x11, 0b00111100);
  }
  void SetAccGyroRate104Hz()
  {
    writeRegister(0x10, 0b01001000);
    writeRegister(0x11, 0b01001100);
  }
};

// Instantiate the new class instead of the old class.
IMUExtended myIMU{Wire, LSM6DS3_ADDRESS};


void setup() 
{
  Serial.begin(115200);
  while (!Serial);
// Starting up/connecting to the IMU
  if (!IMU.begin()) 
  {
    Serial.println("Failed to initialize IMU! Halting.");

    while (1);
  }
  myIMU.SetAccGyroRate13Hz();


  s.reserve(200);

  //Setup for the Distance sensor 
  pinMode(trigPin, OUTPUT);   // The trigger pin will output pulses of electricity.
  pinMode(echoPin, INPUT);    // The echo pin will measure the duration of pulses coming back from the distance sensor.
  //Printing the CSV format: 
  Serial.print("Time"); Serial.print(","); Serial.print("Acceleration x-direct"); Serial.print(","); Serial.println("Distance to object"); 
  // Setup for the display: 


  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) 
  {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }

  // Clear the buffer
  display.clearDisplay();

  display.setTextSize(1);      // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE); // Draw white text
  display.cp437(true);         // Use full 256 char 'Code Page 437' font

  float TempC { 24.33f };
  display.setCursor(10, 10);     // Start at top-left corner
  display.print(TempC);
  display.print(" C");

  display.display();
}

//Print position and velocity on the display 
void PrintPosiAndVelocityDisplay (float posi,float vel) 
{
    display.clearDisplay();
    

    display.setCursor(10, 10);     // Start at top-left corner
    display.print("P: "); display.print(posi); display.print(" m");
    display.setCursor(10, 20);     // Buttom/mid 
    display.print("V: "); display.print(vel); display.print(" m/s");
    display.display(); //Skriver det ud på displayet! 
}

void loop() 
{
  unsigned long StartTimeLoop=millis(); //Used to make a sampling time close to 13 Hz.
  

  float DistToOjbect {0}; 
  IMURead.GetIMUMeasurement(); // Getting the measurements from the IMU 
  
  DistToOjbect=DistMeas.getDistance(); //Getting distance to the object. 

  //Printing The measured values as a CSV file,  
  //Time, Acceleration x direct, distance to object
  Serial.print(millis()); Serial.print(",");Serial.print( IMURead.Measurements.at(0).AccX); Serial.print( IMURead.Measurements.at(0).AccY); Serial.print( IMURead.Measurements.at(0).AccZ); Serial.print(","); Serial.println(DistToOjbect);  
  
  float position {50}; 
  float velocity {2}; 


  
  PrintPosiAndVelocityDisplay(position,velocity); 


  //Calculation the neccesary time to wait, to obtain a sampling frequency of 13 Hz, (and appling the delay ;D)
  unsigned long delayTime=samplingTime-(millis()-StartTimeLoop);
  delay(delayTime);
 


}
