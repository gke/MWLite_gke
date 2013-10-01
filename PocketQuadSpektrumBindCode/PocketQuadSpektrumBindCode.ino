// Code-based Spektrum satellite receiver binding for the HobbyKing Pocket Quad
// Andrew L.
// navigation07@gmail.com

// Notes: 
// 1) The Spektrum satellite receiver comes into the Pocket Quad processor AT32U4 on digital pin 0 (RX) 
// 2) Current MultiWii code supports binding via GUI, however the PocketQuad does not give 
//    the AT32U4 control over the Spektrum satellite receiver power or ground so the MultiWii code as written will never work.
// 3) The MultiWii code could be made to work if anyone can figure out how to detect when something is plugged into pin 0
// 3) The code below is a workaround for people that do not have a Spektrum receiver handy to externally bind the satellite receiver with

// Directions:
// 1) Copy, paste this code into a new Arduino sketch
// 2) Examine the Bind Mode Table below and modify the #define SPEK_BIND_PULSES # to correspond to the mode you would like (I use 2 [1024/22ms] with my DX6I)
// 3) Make sure that Leonardo is set as the board type and that the correct serial port is selected
// 4) Upload this sketch to the PocketQuad over USB
// 5) Unplug the USB cable, then plug in again to reset power
// 6) The satellite receiver should drop into bind mode, proceed as normal from here to bind to your radio
// 7) Once the satellite receiver is successfully bound, open your MultiWii code in Arduino and upload to the PocketQuad over USB
// 8) Your system should be ready to fly.

// Bind Mode Table:
// 2 low pulses: DSM2 1024/22ms 
// 3 low pulses: no result
// 4 low pulses: DSM2 2048/11ms
// 5 low pulses: no result
// 6 low pulses: DSMX 22ms
// 7 low pulses: no result
// 8 low pulses: DSMX 11ms

// Enter your desired number of low pulses 
#define SPEK_BIND_PULSES 2

// Counter Variable
int i = 0;

void setup()
{ 
  
pinMode(0, OUTPUT);

digitalWrite(0, HIGH);
delayMicroseconds(116);

while(i < SPEK_BIND_PULSES) 
{ 
digitalWrite(0, LOW); 
delayMicroseconds(116);
digitalWrite(0, HIGH);
delayMicroseconds(116);
i++;
}

pinMode(0, INPUT);

// End of Setup
}

void loop()
{

}


