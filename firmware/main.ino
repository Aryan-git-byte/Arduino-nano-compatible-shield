#include <Servo.h>
#include <SoftwareSerial.h>
#include <Wire.h>


Servo myservo;
SoftwareSerial bt(12, 11);
unsigned long lastStream = 0;


void moveForward() {
    digitalWrite(2, HIGH);
    digitalWrite(7, LOW);
    digitalWrite(8, HIGH);
    digitalWrite(4, LOW);
    analogWrite(5, 200);
    analogWrite(3, 200);
}

void moveBackward(){
    digitalWrite(7, HIGH);
    digitalWrite(2, LOW);
    digitalWrite(4, HIGH);
    digitalWrite(8, LOW);
    analogWrite(5, 200);
    analogWrite(3, 200);
}

void turnLeft(){
    digitalWrite(2, HIGH);
    digitalWrite(7, LOW);
    digitalWrite(4, HIGH);
    digitalWrite(8, LOW);
    analogWrite(5, 200);
    analogWrite(3, 200);
}

void turnRight(){
    digitalWrite(7, HIGH);
    digitalWrite(2, LOW);
    digitalWrite(8, HIGH);
    digitalWrite(4, LOW);
    analogWrite(5, 200);
    analogWrite(3, 200);
}

void stopmotors(){
    analogWrite(5, 0);
    analogWrite(3, 0);
    digitalWrite(2, LOW);
    digitalWrite(4, LOW);
    digitalWrite(7, LOW);
    digitalWrite(8, LOW);
}

void scanLeft(){
    myservo.write(30);
}

void scanRight(){
    myservo.write(150);
}

void scanCentre(){
    myservo.write(90);
}

void bluetooth(){
    if (bt.available()) {
        char cmd = bt.read();
        switch (cmd) {
            case 'F' : moveForward(); break;
            case 'B': moveBackward(); break;
            case 'L': turnLeft();     break;
            case 'R': turnRight();    break;
            case 'S': stopmotors();   break;
        }
    }
}

long getDistance() {
  digitalWrite(A3, LOW);
  delayMicroseconds(2);
  digitalWrite(A3, HIGH);
  delayMicroseconds(10);       
  digitalWrite(A3, LOW);

  long duration = pulseIn(A6, HIGH, 25000);
  if (duration == 0) return -1;  
  return duration / 58;          
}

void readMPU(int &ax, int &ay, int &az,
             int &gx, int &gy, int &gz,
             float &temp) {
    Wire.beginTransmission(0x68);
    Wire.write(0x3B);             
    Wire.endTransmission(false);
    Wire.requestFrom(0x68, 14);   

    ax = (Wire.read() << 8) | Wire.read();
    ay = (Wire.read() << 8) | Wire.read();
    az = (Wire.read() << 8) | Wire.read();
    int rawTemp = (Wire.read() << 8) | Wire.read();
    gx = (Wire.read() << 8) | Wire.read();
    gy = (Wire.read() << 8) | Wire.read();
    gz = (Wire.read() << 8) | Wire.read();
    temp = rawTemp / 340.0 + 36.53;
}

void setup()
{
    pinMode(2, OUTPUT);
    pinMode(7, OUTPUT);
    pinMode(8, OUTPUT);
    pinMode(4, OUTPUT);
    pinMode(5, OUTPUT);
    pinMode(3, OUTPUT);
    myservo.attach(6);
    pinMode(A3, OUTPUT);
    pinMode(A6, INPUT);
    pinMode(A0, INPUT);
    pinMode(A1, INPUT);
    pinMode(A2, INPUT);
    Serial.begin(9600);
    bt.begin(9600);
    Wire.begin();
    Wire.beginTransmission(0x68);
    Wire.write(0x6B);
    Wire.write(0);
    Wire.endTransmission(true);
    

}
void loop(){
    bluetooth();
    int irfrontleft  = analogRead(A0);
    int irfrontright = analogRead(A1);
    int irback       = analogRead(A2);
    if (irfrontleft < 500 && irfrontright < 500) {
        stopmotors();
    } else if (irfrontleft < 500) {
        turnRight();
        delay(400);
        stopmotors();
    } else if (irfrontright < 500) {
        turnLeft();
        delay(400);
        stopmotors();
    }
    scanCentre();
    long distcenter = getDistance();
    if (distcenter < 20) {
        stopmotors();
        delay(100);

        scanLeft();
        delay(300);
        long distleft = getDistance();
        delay(300);

        scanRight();
        delay(300);
        long distright = getDistance();
        delay(300);

        scanCentre();
        delay(200);

        if (distleft < 20 && distright < 20) {
            if (irback < 500) {
                stopmotors();
            } else {
                moveBackward();
                delay(3000);
                stopmotors();
            }
        }
        else if (distleft > distright) {
            turnLeft();
            delay(400);
            stopmotors();
        }
        else{
            turnRight();
            delay(400);
            stopmotors();
        }
    }
    else {
        moveForward();
    }

    if (millis() - lastStream > 500) {
    lastStream = millis();
    int ax, ay, az, gx, gy, gz;
    float temp;
    readMPU(ax, ay, az, gx, gy, gz, temp);
    bt.print("AX:"); bt.print(ax);
    bt.print(" AY:"); bt.print(ay);
    bt.print(" AZ:"); bt.print(az);
    bt.print(" GX:"); bt.print(gx);
    bt.print(" GY:"); bt.print(gy);
    bt.print(" GZ:"); bt.print(gz);
    bt.print(" TMP:"); bt.println(temp);
}


}