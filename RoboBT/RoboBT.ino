/*
  Software serial multple serial test

 Receives from the hardware serial, sends to software serial.
 Receives from software serial, sends to hardware serial.

 The circuit:
 * RX is digital pin 10 (connect to TX of other device)
 * TX is digital pin 11 (connect to RX of other device)

 Note:
 Not all pins on the Mega and Mega 2560 support change interrupts,
 so only the following can be used for RX:
 10, 11, 12, 13, 50, 51, 52, 53, 62, 63, 64, 65, 66, 67, 68, 69

 Not all pins on the Leonardo support change interrupts,
 so only the following can be used for RX:
 8, 9, 10, 11, 14 (MISO), 15 (SCK), 16 (MOSI).

 created back in the mists of time
 modified 25 May 2012
 by Tom Igoe
 based on Mikal Hart's example

 This example code is in the public domain.

 */
#include <SoftwareSerial.h>

SoftwareSerial mySerial(2, 3); // RX, TX

byte header[2];
byte midder[2];
byte trailer[2];

byte data1[2];
byte data2[2];

void setup()
{
  Serial.begin(115200);
  mySerial.begin(9600);
}

void loop() // run over and over
{
  while (mySerial.available() < 2);
  header[0] = mySerial.read();
  header[1] = mySerial.read();

  if (header[0] == 0x00 && header[1] == 0x2A)
  {
    while (mySerial.available() < 2);
    data1[0] = mySerial.read();
    data1[1] = mySerial.read();

    while (mySerial.available() < 2);
    midder[0] = mySerial.read();
    midder[1] = mySerial.read();

    if (midder[0] == 0x00 && midder[1] == 0x23)
    {
      while (mySerial.available() < 2);
      data2[0] = mySerial.read();
      data2[1] = mySerial.read();

      while (mySerial.available() < 2);
      trailer[0] = mySerial.read();
      trailer[1] = mySerial.read();

      if (trailer[0] == 0x00 && trailer[1] == 0x25)
      {
        //data1 e data2 ok

        int value1 =  ((data1[0] & 0x7f) << 7) | data1[1];
        int value2 =  ((data2[0] & 0x7f) << 7) | data2[1];

        if (bitRead(data1[0], 7) == 1)
          value1 -= 16384;
        if (bitRead(data2[0], 7) == 1)
          value2 -= 16384;

        Serial.println(value1);
        Serial.println(value2);
      }
    }
  }


  if (Serial.available())
    mySerial.write(Serial.read());
}

