
#include <string>
#include <Arduino_LSM6DS3.h>
#include <array>
#include <numeric> 

#include <BasicLinearAlgebra.h>

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>



constexpr struct
{
  size_t echo{2};
  size_t trig{3};
}PinUS;             //Ultrasonic sensor

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
    filterUpToDate=false;
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
    if(!filterUpToDate) filter();
    return(H*x_p);
  }

  void filter()
  {
      y_e = H*x_p;
      y_e = y - y_e;    //Measurement prediction error.
      x_e = x_p+K*y_e;
      x_p= Phi*x_e;

      filterUpToDate=true;
  }

  private: 
    BLA::Matrix<n_states> x_p{0};
    BLA::Matrix<n_states> x_e{0};
    BLA::Matrix<n_output> y_e{0};
    BLA::Matrix<n_output> y{0};

    BLA::Matrix<n_states,n_states>Phi{0};
    BLA::Matrix<n_output,n_states>H{0};
    BLA::Matrix<n_states,n_output>K{0};

    bool filterUpToDate{false};
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
  
    float echoTime = pulseIn(PinUS.echo, HIGH,30*1000);  //[us] 
    
    return(echoTime*timeToDistance ); //Calculate distance [m]

}
  private:
  float timeToDistance {170/1000000.f};  //speed of sound/2 /1 000 000us
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


constexpr struct{
  size_t width {128}; // OLED display width, in pixels
  size_t height {32}; // OLED display height, in pixels
  int OLED_RESET {-1};
  size_t Address {0x3C};
}Screen;

Adafruit_SSD1306 display(Screen.width, Screen.height, &Wire, Screen.OLED_RESET);

void setup() 
{
  Serial.begin(500000);
  while (!Serial);

  // Starting up/connecting to the IMU
  if (!myIMU.begin()) 
  {
    Serial.println("Failed to initialize IMU! Halting.");
    while (true);
  }
  myIMU.SetAccGyroRate26Hz();

  if(!display.begin(SSD1306_SWITCHCAPVCC, Screen.Address)) 
  {
    Serial.println(F("SSD1306 allocation failed"));
    while(true);
  }

  display.clearDisplay();
  display.setTextSize(1);      
  display.setTextColor(SSD1306_WHITE);
  display.cp437(true);         // Use full 256 char 'Code Page 437' font
  display.display();

  
  BLA::Matrix<4,4> Phi={0.99,0,0,0, Tss,1,0,0, 0,Tss,1,0, 0,0,0,1};   //Setting parameters for Height estimator
  HeighEstimator.setPhi(Phi);
  BLA::Matrix<2,4> H ={1,0,0,1, 0,0,1,0};
  HeighEstimator.setH(H);
  BLA::Matrix<4,2> K = {0.12, 2.74, 0.01,1.69, 0,0.43, 0.18,-2.49};
  HeighEstimator.setK(K);
}


void loop() 
{
  unsigned long StartTimeLoop=millis(); //Used to make a sampling time close to 13 Hz.
    
  
  float distance = DistMeas.getDistance();   //Measure distance

  if (myIMU.accelerationAvailable()) 
  {
    myIMU.readAcceleration(Acc.X, Acc.Y, Acc.Z);  //Measure acceleration
  }
  else
  {
    Serial.println("Failed");
  }



  BLA::Matrix<2> measurements{0};   //Inputs to Kalman Filter
  measurements(0)=Acc.X*9.82;
  measurements(1)=distance;
  HeighEstimator.setMeasurements(measurements);

  BLA::Matrix<4>states{0};   //Outputs from Kalman filter
  states= HeighEstimator.getStateEstimate();
  float acc=states(0);
  float vel=states(1);
  float pos=states(2);
  float bia=states(3);


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

  display.clearDisplay();
  display.setCursor(10, 0);     // Start at top-left corner
  display.print("Height "); display.print(pos); display.print(" m");
  display.setCursor(10, 10);     // Buttom/mid 
  display.print("Vel "); display.print(vel); display.print(" m/s");
  display.setCursor(10, 20);     // Buttom/mid 
  display.print("acc "); display.print(acc); display.print(" m/s2");
  display.display();             //Showing data from print 

  signed long delayTime=Ts-(millis()-StartTimeLoop);    //Calculate delay for 13 Hz sampling
  if(delayTime>0) delay(delayTime);
}
