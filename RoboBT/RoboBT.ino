#include <Servo.h>
#include <SoftwareSerial.h>

#define DEBUG 0

//GESTIONE SERVOMOTORI
Servo s_sx, s_dx;
const int sxServoFermo = 1440;
const int dxServoFermo = 1495;
const int tDeltaSx = 100; //900
const int tDeltaDx = 100; //900
const int pinServoSx = 5;
const int pinServoDx = 6;

const int deadAngleZone = 20;
//const int MAX_TIME_LOST_SIGNAL = 200; //min tx Hz -> 1/0.5 + 1 =

unsigned long prevMs = 0UL, currMs = 0UL;

//COMUNICAZIONE BLUETOOTH - SPP
SoftwareSerial mySerial(2, 3); // RX, TX
typedef struct
{
  int x;
  int y;
  boolean isEngineOn;
  boolean isTimedOut;
} Packet;
Packet getPacket();

const byte X00 = 0x00;
const byte X2A = 0x2A;
const byte X23 = 0x23;
const byte X25 = 0X25;

void setup() {
  s_sx.attach(pinServoSx);
  s_dx.attach(pinServoDx);

  s_sx.writeMicroseconds(sxServoFermo);
  s_dx.writeMicroseconds(dxServoFermo);

  Serial.begin(115200);
  mySerial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  Packet pacchetto = getPacket();

  if (!pacchetto.isTimedOut)
  { //se il pacchetto non è andato in time-out ed è quindi corretto
    if (pacchetto.isEngineOn)
    {
      if (!s_sx.attached())
      {
        s_sx.attach(pinServoSx);
        s_dx.attach(pinServoDx);
      }
      setServoSpeed(pacchetto.x, pacchetto.y);
    }
    else
    {
      s_sx.writeMicroseconds(sxServoFermo);
      s_dx.writeMicroseconds(dxServoFermo);
      s_sx.detach();
      s_dx.detach();
    }
  }
/*
  //TIMER LOCK CONNESSIONE IR
  if (currMs - prevMs > MAX_TIME_LOST_SIGNAL)
  {
    prevMs = currMs; //reset
    s_sx.writeMicroseconds(sxServoFermo);
    s_dx.writeMicroseconds(dxServoFermo);
    s_sx.detach();
    s_dx.detach();
    //DEAD ZONE - NULLA
  }
*/
  if (Serial.available())
    mySerial.print((char)Serial.read());
}

void setServoSpeed(int angle_x, int angle_y)
{
  constrain(angle_x, -90, 90);
  constrain(angle_y, -90, 90);

  if (angle_y > deadAngleZone || angle_y < -deadAngleZone || angle_x > deadAngleZone || angle_x < -deadAngleZone)
  { //fuori zona morta

    int forwardSpeed, rotationSpeed;

    //VELOCITA' AVANTI/DIETRO (vettore con punto di applicazione al centro del robot)
    forwardSpeed = map(angle_x, -90 + deadAngleZone, 90 - deadAngleZone, -tDeltaDx, tDeltaDx);

    //VELOCITA' SINISTRA/DESTRA (vettore con punto di applicazione al centro del robot)
    rotationSpeed = map(angle_y, -90 + deadAngleZone, 90 - deadAngleZone, -tDeltaSx, tDeltaSx);

    int sxCalculatedSpeed, dxCalculatedSpeed;

    //VELOCITA' SERVO SX
    sxCalculatedSpeed = map(forwardSpeed - rotationSpeed, -tDeltaSx - tDeltaDx, tDeltaSx + tDeltaDx, -tDeltaSx, tDeltaSx);
    dxCalculatedSpeed = map(forwardSpeed + rotationSpeed, -tDeltaSx - tDeltaDx, tDeltaSx + tDeltaDx, -tDeltaDx, tDeltaDx);

    s_sx.writeMicroseconds(sxServoFermo + sxCalculatedSpeed);
    s_dx.writeMicroseconds(dxServoFermo + dxCalculatedSpeed);

    if (DEBUG)
    {
      Serial.print("forwardSpeed: ");
      Serial.print(forwardSpeed);
      Serial.print(" rotationSpeed: ");
      Serial.print(rotationSpeed);
      Serial.print(" forwardSpeed+rotationSpeed: ");
      Serial.print(forwardSpeed + rotationSpeed);
      Serial.print(" forwardSpeed-rotationSpeed: ");
      Serial.print(forwardSpeed - rotationSpeed);
      Serial.print(" ~ sxCalculatedSpeed: ");
      Serial.print(sxCalculatedSpeed);
      Serial.print(" dxCalculatedSpeed: ");
      Serial.println(dxCalculatedSpeed);
    }
  }
  else
  {
    if (DEBUG)
      Serial.print("fermo");
      
    s_sx.writeMicroseconds(sxServoFermo);
    s_dx.writeMicroseconds(dxServoFermo);
  }
}

Packet getPacket()
{
  //timer < 20 ms max
  Packet packet;
  packet.isTimedOut = true;

  prevMs = currMs = millis(); //reset

  while (currMs - prevMs < 10)
  {
    if (mySerial.available() > 9)
    {
      if (mySerial.peek() == X00)
      {
        if (mySerial.read() == X00 && mySerial.read() == X2A)
        {
          byte n1[2];
          n1[0] = mySerial.read();
          n1[1] = mySerial.read();
          if (mySerial.read() == X00 && mySerial.read() == X23)
          {
            byte n2[2];
            n2[0] = mySerial.read();
            n2[1] = mySerial.read();
            if (mySerial.read() == X00 && mySerial.read() == X25)
            {
              //all ok!
              packet.isTimedOut = false;

              packet.x = ((n1[0] & 0x7f) << 7) | n1[1];
              packet.y =  ((n2[0] & 0x7f) << 7) | n2[1];

              if (bitRead(n1[0], 7) == 1)
                packet.x -= 16384;
              if (bitRead(n2[0], 7) == 1)
                packet.y -= 16384;

              Serial.print(packet.x);
              Serial.print(" - ");
              Serial.println(packet.y);
              Seruak
              return packet;
            }
          }
        }
      }
      else
      {
        mySerial.read();
      }
    }
    currMs = millis();
  }

  return packet;
}
