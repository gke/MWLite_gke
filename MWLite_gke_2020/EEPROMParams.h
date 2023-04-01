#if defined(USE_MW)

#if defined(MULTICOPTER)

conf.pid[ROLL].Kp = conf.pid[PITCH].Kp = 33;  
conf.pid[ROLL].Ki = conf.pid[PITCH].Ki = 30; 
conf.pid[ROLL].Kd = conf.pid[PITCH].Kd = 23;

conf.pid[LEVEL].Kp = 90; 
conf.pid[LEVEL].Ki = 10; 
conf.pid[LEVEL].Kd = 100;

conf.rates.rcRate8 = 80; 
conf.rates.rcExpo8 = 50;

conf.rates.rollPitchRate = 60;
conf.rates.yawRate = 100;

conf.pid[YAW].Kp = 80;  
conf.pid[YAW].Ki = 40;
conf.pid[YAW].Kd = 60;

conf.rates.thrMid8 = 50; 
conf.rates.thrExpo8 = 0; 

conf.pid[ALTITUDE].Kp = 64; 
conf.pid[ALTITUDE].Ki = 25; 
conf.pid[ALTITUDE].Kd = 24;

conf.pid[HEADING].Kp   = 40;

conf.rates.dynThrPID = 0;

#else

#error "MultiWii Control only supports multicopter."

#endif

#else

#if defined(MULTICOPTER)

#if defined(WOLFERL)

conf.pid[ROLL].Kp = conf.pid[PITCH].Kp = 35;   
conf.pid[ROLL].Ki = conf.pid[PITCH].Ki = 60; 
conf.pid[ROLL].Kd = conf.pid[PITCH].Kd = 25;

conf.P8[LEVEL] = 90; 
conf.pid[LEVEL].Ki = 1; 
conf.pid[LEVEL].Kd = 100;

#else

conf.pid[ROLL].Kp = conf.pid[PITCH].Kp = 35;   
conf.pid[ROLL].Kd = conf.pid[PITCH].Kd = 25;

conf.pid[LEVEL].Kp = 30; // 50 oscillates
conf.pid[LEVEL].Ki = 20; // 1

#endif

conf.pid[YAW].Kp = 68;  
conf.pid[YAW].Ki = 45; 

conf.pid[HEADING].Kp = 40; 

conf.rates.rcRate8 = 100; 
conf.rates.yawRate = 100;

conf.rates.thrMid8 = 50; 
conf.rates.thrExpo8 = 0; 

conf.pid[ALTITUDE].Kp = 64; 
conf.pid[ALTITUDE].Ki = 25; 
conf.pid[ALTITUDE].Kd = 24;

//________________________

#elif defined(FLYING_WING)

conf.pid[ROLL].Kp = 15; // 35;   
conf.pid[ROLL].Kd = 10;

conf.pid[PITCH].Kp = 15; // 35;  
conf.pid[PITCH].Kd = 10;

conf.pid[YAW].Kp = 10;  
conf.pid[YAW].Ki = 0; 

conf.pid[LEVEL].Kp = 10;
conf.pid[LEVEL].Ki = 0;//10; 

conf.pid[HEADING].Kp = 40; 

conf.rates.rcRate8 = 100; 
conf.rates.yawRate = 0; 

conf.rates.thrMid8 = 50; 
conf.rates.thrExpo8 = 0; 

conf.pid[ALTITUDE].Kp = 64; 
conf.pid[ALTITUDE].Ki = 25; 
conf.pid[ALTITUDE].Kd = 24;

//________________________

#elif defined(AIRPLANE) || defined(VTAIL) 

conf.pid[ROLL].Kp = 15;   
conf.pid[ROLL].Kd = 25;

conf.pid[PITCH].Kp = 8;   
conf.pid[PITCH].Kd = 12;

conf.pid[YAW].Kp = 80;  
conf.pid[YAW].Ki = 25;

conf.P8[LEVEL] = 40; 
conf.pid[LEVEL].Ki = 10; 

conf.pid[HEADING].Kp = 40; 

conf.rcRate8 = 100; 
conf.yawRate = 100;

conf.thrMid8 = 50; 
conf.thrExpo8 = 0; 

conf.pid[ALTITUDE].Kp = 64; 
conf.pid[ALTITUDE].Ki = 25; 
conf.pid[ALTITUDE].Kd = 24;

#else

#error "Aircraft type not defined"

#endif 

// PI/PID final position 
conf.pid[POSITION].Kp = 50;     
conf.pid[POSITION].Ki = 0;       

conf.pid[POSITIONRATE].Kp = 1; 
conf.pid[POSITIONRATE].Ki = 0;  
conf.pid[POSITIONRATE].Kd = 0;

// PID approach velocity
conf.pid[NAVRATE].Kp = 1;          
conf.pid[NAVRATE].Ki = 0;           
conf.pid[NAVRATE].Kd = 0;

conf.pid[VELOCITY].Kp = 1;      
conf.pid[VELOCITY].Ki = 0;    
conf.pid[VELOCITY].Kd = 0;

#endif



















