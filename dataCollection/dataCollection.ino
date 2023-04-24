
#include <string>
#include <Arduino_LSM6DS3.h>
#include <array>
#include <numeric> 



constexpr struct
{
  size_t echo{2};
  size_t trig{3};
}PinUS;

size_t samplingTime {77}; // Time between sample should be taken in ms. 

class HCSR04US
{
  public:

  HCSR04US()
  {
    pinMode(PinUS.trig,OUTPUT);
    pinMode(PinUS.echo,INPUT);
  }
  
  float getDistance()
  {
    // Send out an ultrasonic pulse that's 10 us long.
    digitalWrite(PinUS.trig, HIGH);
    delayMicroseconds(10);
    digitalWrite(PinUS.trig, LOW);
  
    // Use the pulsein command to see how long it takes for the pulse to bounce back to the sensor.
    float echoTime = pulseIn(PinUS.echo, HIGH,30*1000);     
    

    // Calculate the distance of the object that reflected the pulse (half the bounce time multiplied by the speed of sound).
    return(echoTime*timeToDistance ); //Regner afstand til et objekt i meter! 

}

  private:
  float timeToDistance {170/1000.f};
};

HCSR04US DistMeas; 



struct
{
  float X{0};
  float Y{0};
  float Z{0};
}Acc;

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

IMUExtended myIMU{Wire,LSM6DS3_ADDRESS};




void setup() 
{
  Serial.begin(500000);
  while (!Serial);

// Starting up/connecting to the IMU
  if (!myIMU.begin()) 
  {
    Serial.println("Failed to initialize IMU! Halting.");

    while (1);
  }
  myIMU.SetAccGyroRate26Hz();

}


void loop() 
{
  unsigned long StartTimeLoop=millis(); //Used to make a sampling time close to 13 Hz.
    
  

  
  if (myIMU.accelerationAvailable()) 
  {
    myIMU.readAcceleration(Acc.X, Acc.Y, Acc.Z);
  }
  else
  {
    Serial.println("Failed");
  }       

  
  Serial.print(millis()); 
  Serial.print(", ");
  Serial.print( Acc.X); 
  Serial.print(", ");
  Serial.print( Acc.Y); 
  Serial.print(", ");
  Serial.print( Acc.Z); 
  Serial.print(", "); 
  Serial.println(DistMeas.getDistance());  
  

  //Calculation the neccesary time to wait, to obtain a sampling frequency of 13 Hz, (and appling the delay ;D)
  signed long delayTime=samplingTime-(millis()-StartTimeLoop);
  if(delayTime>0) delay(delayTime);
 
}
