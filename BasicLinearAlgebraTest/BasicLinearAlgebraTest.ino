#include <BasicLinearAlgebra.h>

BLA::Matrix<3> v={1, 2, 3};
BLA::Matrix<1,3> vT;

BLA::Matrix<3, 3> B = {6.54, 3.66, 2.95, 3.22, 7.54, 5.12, 8.98, 9.99, 1.56};




void setup() {
  
  // put your setup code here, to run once:
  Serial.begin(9600);
  delay(500);

  vT.Fill(0);



}

void loop() {
  // put your main code here, to run repeatedly:
  
  Serial.println(v(1));
  vT= ~v;
  auto a = vT*v;
  float b = a(0);
  
  Serial.println(b);
  delay(200);
  
  Serial << "B: " << B << '\n';
  

}
