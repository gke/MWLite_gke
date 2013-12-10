/*

 MWLite_gke
 May 2013
 
 MWLite_gke is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 any later version. see <http://www.gnu.org/licenses/>
 
 MWLite_gke is distributed in the hope that it will be useful,but WITHOUT ANY 
 WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR 
 A PARTICULAR PURPOSE. 
 
 See the GNU General Public License for more details.
 
 Lite was based originally on MultiWiiCopter V2.2 by Alexandre Dubus
 www.multiwii.com, March  2013. The rewrite by Prof Greg Egan was renamed 
 so as not to confuse it with the original.
 
 It preserves the parameter specification and GUI interface with parameters
 scaled to familiar values. 
 
 Major changes include the control core which comes from UAVX with the 
 addition of MW modes.
 
 Lite supports only Atmel 32u4 processors using an MPU6050 and optionally 
 BMP085 and MS5611 barometers and HMC5883 magnetometer with 4 motors, 
 no servos and 8KHz PWM for brushed DC motors.
 
 */

//________________________

#if defined(USE_MW)

conf.P8[ROLL] = conf.P8[PITCH] = 33;  
conf.I8[ROLL] = conf.I8[PITCH] = 30; 
conf.D8[ROLL] = conf.D8[PITCH] = 23;

conf.P8[PIDLEVEL] = 90; 
conf.I8[PIDLEVEL] = 10; 
conf.D8[PIDLEVEL] = 100;

conf.rcRate8 = 80; 
conf.rcExpo8 = 50;

conf.rollPitchRate = 60;
conf.yawRate = 100;

conf.P8[YAW] = 80;  
conf.I8[YAW] = 40;
conf.D8[YAW] = 60;

#if defined(USE_THROTTLE_CURVE)
conf.thrMid8 = 45; 
conf.thrExpo8 = 40;
#else  
conf.thrMid8 = 50; 
conf.thrExpo8 = 0; 
#endif // USE_THROTTLE_CURVE

conf.P8[PIDALT] = 64; 
conf.I8[PIDALT] = 25; 
conf.D8[PIDALT] = 24;

conf.P8[PIDMAG]   = 40;

conf.dynThrPID = 0;

#else // !MW

//________________________

#if defined(ISMULTICOPTER)

#if defined(WOLFERL)

conf.P8[ROLL] = conf.P8[PITCH] = 35;   
conf.I8[ROLL] = conf.I8[PITCH] = 60; 
conf.D8[ROLL] = conf.D8[PITCH] = 25;

conf.P8[PIDLEVEL] = 90; 
conf.I8[PIDLEVEL] = 1; 
conf.D8[PIDLEVEL] = 100;

#else

conf.P8[ROLL] = conf.P8[PITCH] = 35;   
conf.D8[ROLL] = conf.D8[PITCH] = 25;

conf.P8[PIDLEVEL] = 50;
conf.I8[PIDLEVEL] = 10; 

#endif

conf.P8[YAW] = 80;  
conf.I8[YAW] = 25; 

conf.P8[PIDMAG] = 40; 

conf.rcRate8 = 100; 
conf.yawRate = 100;

#if defined(USE_THROTTLE_CURVE)
conf.thrMid8 = 45; 
conf.thrExpo8 = 40;
#else  
conf.thrMid8 = 50; 
conf.thrExpo8 = 0; 
#endif // USE_THROTTLE_CURVE

conf.P8[PIDALT] = 64; 
conf.I8[PIDALT] = 25; 
conf.D8[PIDALT] = 24;

#else

//________________________

#if defined(FLYING_WING)

conf.P8[ROLL] = 35;   
conf.D8[ROLL] = 25;

conf.P8[PITCH] = 35;  
conf.D8[PITCH] = 25;

conf.P8[YAW] = 80;  
conf.I8[YAW] = 25; 

conf.P8[PIDLEVEL] = 90;
conf.I8[PIDLEVEL] = 10; 

conf.P8[PIDMAG] = 40; 

conf.rcRate8 = 100; 
conf.yawRate = 0;

//________________________

#elif defined(AIRPLANE) || defined(VTAIL) 

conf.P8[ROLL] = 35;   
conf.D8[ROLL] = 25;

conf.P8[PITCH] = 8;   
conf.D8[PITCH] = 12;

conf.P8[YAW] = 80;  
conf.I8[YAW] = 25;

conf.P8[PIDLEVEL] = 40; 
conf.I8[PIDLEVEL] = 10; 

conf.P8[PIDMAG] = 40; 

conf.rcRate8 = 100; 
conf.yawRate = 100;

#else

#error "Aircraft type not defined"

#endif 

conf.thrMid8 = 50; 
conf.thrExpo8 = 0; 

conf.P8[PIDALT] = 64; 
conf.I8[PIDALT] = 25; 
conf.D8[PIDALT] = 24;

#endif

#endif

















