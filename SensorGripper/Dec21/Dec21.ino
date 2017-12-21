int x, y, z,xright, yright, zright;
double time1;

void setup()
{
  Serial.begin(250000);      // sets the serial port to 9600
}

void loop()
{
  time1 = millis();
  x = analogRead(0);       // read analog input pin 0
  y = analogRead(1);       // read analog input pin 1
  z = analogRead(2);       // read analog input pin 1
  xright= analogRead(3);  
  yright= analogRead(4);  
  zright= analogRead(5);  
 // Serial.print("accelerations are x, y, z: ");
   
  Serial.print(time1);  // print the acceleration in the Z axis
   Serial.print(" ");       // prints a space between the numbers
  Serial.print(x, DEC);    // print the acceleration in the X axis
  Serial.print(" ");       // prints a space between the numbers
  Serial.print(y, DEC);    // print the acceleration in the Y axis
  Serial.print(" ");       // prints a space between the numbers
  Serial.println(z, DEC);  // print the acceleration in the Z axis
 // Serial.print(" ");   
  //Serial.print(xright, DEC);    // print the acceleration in the X axis
  //Serial.print(" ");       // prints a space between the numbers
  //Serial.print(yright, DEC);    // print the acceleration in the Y axis
  //Serial.print(" ");       // prints a space between the numbers
  //Serial.print(zright, DEC);  // print the acceleration in the Z axis

 // delay(10);              // wait 100ms for next reading

  //  long reading = 0;
 // analogRead(axisPin);
 // delay(1);
 // for (int i = 0; i < sampleSize; i++)
 // {
 //   reading += analogRead(axisPin);
 // }
  //return reading/sampleSize;
}


