
#include <string>
#include <Arduino_LSM6DS3.h>
#include <array>
#include <numeric> 

#include <BasicLinearAlgebra.h>



constexpr struct
{
  size_t echo{2};
  size_t trig{3};
}PinUS;

float Ts {77.f}; // Time between sample should be taken in ms. 
float Tss=Ts/1000;


template<size_t n_states,size_t n_output>
class KalmanFilter
{
  public:

  void setPhi(BLA::Matrix<n_states,n_states> Phi)
  {
    this->Phi = Phi;
  }

  void setH(BLA::Matrix<n_output,n_states>H)
  {
    this->H = H;
  }

  void setK(BLA::Matrix<n_states,n_output>K)
  {
    this->K = K;
  }

  void setMeasurements(BLA::Matrix<n_output>y)
  {
    this->y=y;
  }

  BLA::Matrix<n_states> getStateEstimate()
  {
    if(!filterUpToDate) filter();
    return(x_e);
  }

  BLA::Matrix<n_states> getStatePrediction()
  {
    if(!filterUpToDate) filter();
    return(x_p);
  }

  BLA::Matrix<n_output> getMeasurementPrediction()
  {
    return(H*x_p);
  }

  private: 
    BLA::Matrix<n_states> x_p{0};
    BLA::Matrix<n_states> x_e{0};
    BLA::Matrix<n_output> y_e{0};
    BLA::Matrix<n_output> y_error{0};
    BLA::Matrix<n_output> y{0};

    BLA::Matrix<n_states,n_states>Phi{0};
    BLA::Matrix<n_output,n_states>H{0};

    BLA::Matrix<n_states,n_output>K{0};

    bool filterUpToDate{false};

    void filter()
    {
      y_e = H*x_p;
      y_error = y - y_e;
      x_e = x_p+K*y_error;
      x_p= Phi*x_e;

      filterUpToDate=true;
    }
};

KalmanFilter<4,2> HeighEstimator;


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

  
  BLA::Matrix<4,4> Phi={0.99,0,0,0, Tss,1,0,0, 0,Tss,1,0, 0,0,0,1};
  HeighEstimator.setPhi(Phi);
  BLA::Matrix<2,4> H ={1,0,0,1, 0,0,1,0};
  HeighEstimator.setH(H);
  BLA::Matrix<4,2> K = {0.2, 1.18, 0.03,1.12, 0,0.36, 0,-0.07};
  HeighEstimator.setK(K);
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

  float distance = DistMeas.getDistance()/1000;


  BLA::Matrix<2> measurements{0};
  BLA::Matrix<4> states{0};
  measurements(0)=Acc.X*9.82;
  measurements(1)=distance;
  HeighEstimator.setMeasurements(measurements);
  states= HeighEstimator.getStateEstimate();
  float acc=states(0);
  float vel=states(1);
  float pos=states(2);
  float bia=states(3);

  //Calculation the neccesary time to wait, to obtain a sampling frequency of 13 Hz, (and appling the delay ;D)
  signed long delayTime=Ts-(millis()-StartTimeLoop);
  if(delayTime>0) delay(delayTime);

  Serial.print(millis()); 
  Serial.print(", ");
  Serial.print( Acc.X*9.82); 
  Serial.print(", ");
  Serial.print( Acc.Y*9.82); 
  Serial.print(", ");
  Serial.print( Acc.Z*9.82); 
  Serial.print(", "); 
  Serial.print(distance);  
  Serial.print(", ");
  Serial.print( acc); 
  Serial.print(", ");
  Serial.print( vel); 
  Serial.print(", ");
  Serial.print( pos); 
  Serial.print(", "); 
  Serial.print(bia); 
  Serial.print(", ");
  Serial.println(delayTime);
 
}
