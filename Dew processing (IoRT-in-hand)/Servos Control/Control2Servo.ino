#include <Servo.h>
char buffer[11];
Servo servo1; // Create a servo object
Servo servo2; // Create a second servo object
void setup()
{
  servo1.attach(3); // Attaches the servo on pin 5 to the
  servo2.attach(2); // Attaches the servo on pin 6 to the
  Serial.begin(38400);
  while(Serial.available())
  Serial.read();
  servo1.write(90); // Put servo1 at home position
  servo2.write(90); // Put servo2 at home postion
  //Serial.println("STARTING...");
}
void loop()
{
  if (Serial.available() > 0) { 
    int index=0;
    delay(100); // Let the buffer fill up
    int numChar = Serial.available(); 
    if (numChar>10) {
      numChar=10;
    }
    while (numChar--) {
      // Fill the buffer with the string
      buffer[index++] = Serial.read();
    }
    buffer[index]='\0';
    splitString(buffer); // Run splitString function
  }
}

void splitString(char* data) {
  //Serial.print("Data entered: ");
  //Serial.println(data);
  char* parameter;
  parameter = strtok (data, " ,"); //String to token
  while (parameter != NULL) { 
    setServo(parameter); // ...run the setServo function
    parameter = strtok (NULL, " ,");
  }
  while(Serial.available())
  Serial.read();
}

/** 
 *  W: Wrist
 *  B: Base
*/
void setServo(char* data) {
  if ((data[0] == 'W')) {
    int firstVal = strtol(data+1, NULL, 10); 
    firstVal = constrain(firstVal,0,180); 
    servo1.write(firstVal);
    //Serial.print("Servo1 is set to: ");
    //Serial.println(firstVal);
  }
  if ((data[0] == 'B')) {
    int secondVal = strtol(data+1, NULL, 10); 
    secondVal = constrain(secondVal,0,255); 
    servo2.write(secondVal);
    //Serial.print("Servo2 is set to: ");
    //Serial.println(secondVal);
  }
}
