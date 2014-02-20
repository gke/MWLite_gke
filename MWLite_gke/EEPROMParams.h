#if defined(USE_MW)

#if defined(MULTICOPTER)

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

conf.thrMid8 = 50; 
conf.thrExpo8 = 0; 

conf.P8[PIDALT] = 64; 
conf.I8[PIDALT] = 25; 
conf.D8[PIDALT] = 24;

conf.P8[PIDMAG]   = 40;

conf.dynThrPID = 0;

#else

#error "MultiWii Control only supports multicopter."

#endif

#else

#if defined(MULTICOPTER)

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

conf.P8[PIDLEVEL] = 30; // 50 oscillates
conf.I8[PIDLEVEL] = 20; // 1

#endif

conf.P8[YAW] = 68;  
conf.I8[YAW] = 45; 

conf.P8[PIDMAG] = 40; 

conf.rcRate8 = 100; 
conf.yawRate = 100;

conf.thrMid8 = 50; 
conf.thrExpo8 = 0; 

conf.P8[PIDALT] = 64; 
conf.I8[PIDALT] = 25; 
conf.D8[PIDALT] = 24;

//________________________

#elif defined(FLYING_WING)

conf.P8[ROLL] = 25; // 35;   
conf.D8[ROLL] = 10;

conf.P8[PITCH] = 25; // 35;  
conf.D8[PITCH] = 10;

conf.P8[YAW] = 15;  
conf.I8[YAW] = 0; 

conf.P8[PIDLEVEL] = 10;
conf.I8[PIDLEVEL] = 0;//10; 

conf.P8[PIDMAG] = 40; 

conf.rcRate8 = 100; 
conf.yawRate = 0; 

conf.thrMid8 = 50; 
conf.thrExpo8 = 0; 

conf.P8[PIDALT] = 64; 
conf.I8[PIDALT] = 25; 
conf.D8[PIDALT] = 24;

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

conf.thrMid8 = 50; 
conf.thrExpo8 = 0; 

conf.P8[PIDALT] = 64; 
conf.I8[PIDALT] = 25; 
conf.D8[PIDALT] = 24;

#else

#error "Aircraft type not defined"

#endif 

// PI/PID final position 
conf.P8[PIDPOS] = 50;     
conf.I8[PIDPOS] = 0;       

conf.P8[PIDPOSR] = 0; 
conf.I8[PIDPOSR] = 0;  
conf.D8[PIDPOSR] = 0;

// PID approach velocity
conf.P8[PIDNAVR] = 0;          
conf.I8[PIDNAVR] = 0;           
conf.D8[PIDNAVR] = 0;

conf.P8[PIDVEL] = 0;      
conf.I8[PIDVEL] = 0;    
conf.D8[PIDVEL] = 0;

#endif



















