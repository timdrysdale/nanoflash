//Firmware for MiniTruss remote lab experiment
// Single controller firmware with state machine for reading and writing.

//Author: Timothy D. Drysdale
// timothy.d.drysdale@gmail.com
// 05/09/2022

// IMPORT LIBRARIES
#include "ArduinoJson-v6.9.1.h"
#include <FlashStorage.h>

//JSON serialization
#define COMMAND_SIZE 192  //https://arduinojson.org/v6/assistant
StaticJsonDocument<COMMAND_SIZE> doc;
char command[COMMAND_SIZE];

#define FLASH_WRITES_MAX 20
#define SECRET_LEN_MAX 99
#define SCALE_FACTOR_LEN 7
const size_t SCALE_FACTOR_CAPACITY = JSON_ARRAY_SIZE(SCALE_FACTOR_LEN);
StaticJsonDocument<SCALE_FACTOR_CAPACITY> scale_factors;

// calibration value indices in cal array
#define LOAD_CELL 0
#define MEMBER_1 1
#define MEMBER_2 2
#define MEMBER_3 3
#define MEMBER_4 4
#define MEMBER_5 5
#define MEMBER_6 6

// Create a structure that contains is big enough to contain a name
// and a surname. The "valid" variable is set to "true" once
// the structure is filled with actual data for the first time.
typedef struct {
  boolean secure;          // Set this true when secret is first written 
  boolean valid;           // Set this true when calibration data is first written
  char secret[SECRET_LEN_MAX];  // Secret string for authorising calibration updates (typically a uuid of 36 chars in form 8-4-4-4-12)
  int  writes;             // Count number of remaining writes we'll permit
  // You can change the values below here to suit your experiment
  float scale_factor[SCALE_FACTOR_LEN];   // Scale factors for some experiment or other
} Calibration;

// Create a global "Calibration" variable and call it cal
Calibration cal;

// Reserve a portion of flash memory to store a "Calibration" and
// call it "cal_store".
FlashStorage(cal_store, Calibration);


// Timer setup for reporting
bool do_report;
float report_interval = 500; //ms, for timer interrupt

// Timer (specific to nano IOT 33)
#define CPU_HZ 48000000
#define TIMER_PRESCALER_DIV 1024
float timer_interrupt_freq = 1000.0/report_interval;

// cal function prototypes

bool cal_is_secure();
bool cal_is_valid();
void startTimer(int);
void report();
void cal_set_secret(const char *);

/**
 * Defines the valid states for the state machine
 * 
 */
typedef enum
{
  STATE_STARTUP = 0,        // anything to do with starting up the experiment
  STATE_SETSECRET = 1,     // await authorisation setup before accepting calibration data
  STATE_SETCAL = 2,       // await a calibration data before beginning normal operation of the experiment
  STATE_STANDBY = 3,        // Standby for commands relating to normal operation - represents all the states that would normally be associated with normal operation
  STATE_LOADCAL = 4,      // update calibration
} StateType;

//state Machine function prototypes
//these are the functions that run whilst in each respective state.
void Sm_State_StartUp(void);
void Sm_State_SetSecret(void);
void Sm_State_SetCal(void);
void Sm_State_Standby(void);
void Sm_State_LoadCal(void);

/**
 * Type definition used to define the state
 */
 
typedef struct
{
  StateType State; /**< Defines the command */
  void (*func)(void); /**< Defines the function to run */
} StateMachineType;

/**
 * A table that defines the valid states of the state machine and
 * the function that should be executed for each state
 */
StateMachineType StateMachine[] =
{
  {STATE_STARTUP,     Sm_State_StartUp},
  {STATE_SETSECRET,   Sm_State_SetSecret}, 
  {STATE_SETCAL,      Sm_State_SetCal},
  {STATE_STANDBY,     Sm_State_Standby},
  {STATE_LOADCAL,     Sm_State_LoadCal}, 
};
 
int NUM_STATES = 5;

/**
 * Stores the current state of the state machine
 */
 
StateType SmState = STATE_STARTUP;    // Start off in StartUp state 

//DEFINE STATE MACHINE FUNCTIONS================================================================


//TRANSITION: STATE_STARTUP -> STATE_SECURECAL (if calibration is secure and valid)
void Sm_State_StartUp(void){

    // do any other start up tasks required before checking/loading calibration here
    
    SmState = STATE_SETSECRET;

}

//TRANSITION: STATE_SETSECRET -> STATE_SETCAL (if cal is secure)
void Sm_State_SetSecret(void){

  if (cal_is_secure()) {
    SmState = STATE_SETCAL; //we let startup state dictate progress through essential tasks
    }

  SmState = STATE_SETSECRET; 
  
}


//TRANSITION: STATE_SETCAL -> STATE_LOADCAL (if cal is set)
void Sm_State_SetCal(void){

  if (!cal_is_valid()) {
    SmState = STATE_LOADCAL;
    }

  SmState = STATE_SETCAL; 
  
}
//TRANSITION: STATE_LOADCAL -> STATE_STANDBY (if cal is set)
void Sm_State_LoadCal(void){

   
  
}

//TRANSITION: STATE_STANDBY -> STATE_STANDBY
void Sm_State_Standby(void){

  SmState = STATE_STANDBY;
  
}


//STATE MACHINE RUN FUNCTION
void Sm_Run(void)
{
  if (SmState < NUM_STATES)
  {
    SmState = readSerialJSON(SmState);      
    (*StateMachine[SmState].func)();        //reads the current state and then runs the associated function
    
  }
  else{
    Serial.println("Exception in State Machine");
  }
  
}



void setup() {

  startTimer(timer_interrupt_freq);   //setup and start the timer interrupt functions for PID calculations

  //Serial communication for sending data -> RPi -> Server
  Serial.begin(57600);
  while(!Serial);



}

void loop() {
    Sm_Run();

    // reporting
    if (do_report) {
        report();
        do_report = false;
      }
}






StateType readSerialJSON(StateType SmState){
  if(Serial.available() > 0)
  {
    
    Serial.readBytesUntil(10, command, COMMAND_SIZE);
    deserializeJson(doc, command);
    
    const char* set = doc["set"];

    if(strcmp(set, "secret")==0)
    {
  
        const char* secret = doc["to"]; 

        cal_set_secret(secret); // this function can be safely called many times because it ignores all but the first non-empty secret 
   } 
    else if(strcmp(set, "cal")==0)
    {

      cal = cal_store.read();

      if (!cal.secure) {
        Serial.println("{\"error\":\"cal secret not set\"}");
        return SmState; // don't set values before setting authorisation (prevent rogue writes)
      }
      const char* secret =  doc["auth"];

      
      if (!(strcmp(cal.secret, secret)==0)) {
        Serial.print("{\"error\":\"wrong secret\",");
        Serial.print("\"want\":\"");
        Serial.print(cal.secret); // do NOT do this in production! 
        Serial.print("\",\"have\":\"");
        Serial.print(secret);
        Serial.println("\",\"warning\":\"you are revealing secrets - do not release this code into production\"}");
          return SmState; // don't set values if auth code does not match secret
        } 

      JsonArray values = doc["to"];

      if (values.size() != SCALE_FACTOR_LEN) { 
        Serial.print("{\"error\":\"wrong number of values in cal array\",");
        Serial.print("\"want\":");
        Serial.print(SCALE_FACTOR_LEN);
        Serial.print(",\"have\":");
        Serial.print(values.size());
        Serial.println("}");
        return SmState; // don't set cal values if wrong number 
        } //size ok
        
        Serial.print("{\"log\":\"cal\",\"is\":\"ok\",\"values\":[");
        for (int i=0; i<SCALE_FACTOR_LEN; i++) {
            cal.scale_factor[i]= values[i];
            Serial.print(cal.scale_factor[i]);
            if (i<(SCALE_FACTOR_LEN-1)){
            Serial.print(",");
            } else
            Serial.println("]}");
          } // for
          
        //cal.scale_factor = values;
        
        cal.valid = true;
        cal.writes -= 1;
        cal_store.write(cal);
           
        
     } // do cal
    
  }
      return SmState;     //return whatever state it changed to or maintain the state.
 } 


void report(){
  Serial.print("{\"secure\":");
  Serial.print(cal.secure);
  Serial.print(",\"valid\":");
  Serial.print(cal.valid);
  Serial.print(",\"writes_left\":");
  Serial.print(cal.writes);  
  Serial.print(",\"secret\":"); //do NOT reveal secret over serial comms in production - for testing/demo only.
  Serial.print(cal.secret); 
  Serial.print(",\"sf_load\":"); 
  Serial.print(cal.scale_factor[LOAD_CELL]);
  Serial.print(",\"sf_m1\":"); 
  Serial.print(cal.scale_factor[MEMBER_1]); 
  Serial.print(",\"sf_m2\":"); 
  Serial.print(cal.scale_factor[MEMBER_2]);  
  Serial.print(",\"sf_m3\":"); 
  Serial.print(cal.scale_factor[MEMBER_3]); 
  Serial.print(",\"sf_m4\":"); 
  Serial.print(cal.scale_factor[MEMBER_4]);      
  Serial.print(",\"sf_m5\":"); 
  Serial.print(cal.scale_factor[MEMBER_5]); 
  Serial.print(",\"sf_m6\":"); 
  Serial.print(cal.scale_factor[MEMBER_6]);    
  Serial.print(cal.valid); 
  Serial.println("}");
 
}

bool cal_is_secure(){

    cal = cal_store.read();
  
    return cal.secure;
    
  }

bool cal_is_valid(){

    cal = cal_store.read();
 
    return (cal.secure && cal.valid);
    
  }
  
// set the secret for authorising changes to calibration data
void cal_set_secret(const char *secret){

    if (secret[0] == '\0') { 
      return; //empty string
    }
  
    cal = cal_store.read();
    
    if (!cal.secure){ // only set secret once

        strncpy(cal.secret, secret, (sizeof cal.secret) - 1);
        cal.secure = true;
        cal_store.write(cal);
      }
  }


bool load_cal(){
  
    cal = cal_store.read();
    
  }
 
//This function is run on a timer interrupt defined by PIDInterval/timer_interrupt_freq.
void TimerInterrupt(void) {
   do_report = true;
}

//===================================================================================
//======================TIMER INTERRUPT FUNCTIONS====================================
//      FROM https://github.com/nebs/arduino-zero-timer-demo/
//===================================================================================

void setTimerFrequencyHz(int frequencyHz) {
  int compareValue = (CPU_HZ / (TIMER_PRESCALER_DIV * frequencyHz)) - 1;
  TcCount16* TC = (TcCount16*) TC3;
  // Make sure the count is in a proportional position to where it was
  // to prevent any jitter or disconnect when changing the compare value.
  TC->COUNT.reg = map(TC->COUNT.reg, 0, TC->CC[0].reg, 0, compareValue);
  TC->CC[0].reg = compareValue;
  while (TC->STATUS.bit.SYNCBUSY == 1);
}

/*
This is a slightly modified version of the timer setup found at:
https://github.com/maxbader/arduino_tools
 */
void startTimer(int frequencyHz) {
  REG_GCLK_CLKCTRL = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID (GCM_TCC2_TC3)) ;
  while ( GCLK->STATUS.bit.SYNCBUSY == 1 );

  TcCount16* TC = (TcCount16*) TC3;

  TC->CTRLA.reg &= ~TC_CTRLA_ENABLE;

  // Use the 16-bit timer
  TC->CTRLA.reg |= TC_CTRLA_MODE_COUNT16;
  while (TC->STATUS.bit.SYNCBUSY == 1);

  // Use match mode so that the timer counter resets when the count matches the compare register
  TC->CTRLA.reg |= TC_CTRLA_WAVEGEN_MFRQ;
  while (TC->STATUS.bit.SYNCBUSY == 1);

  // Set prescaler to 1024
  TC->CTRLA.reg |= TC_CTRLA_PRESCALER_DIV1024;
  while (TC->STATUS.bit.SYNCBUSY == 1);

  setTimerFrequencyHz(frequencyHz);

  // Enable the compare interrupt
  TC->INTENSET.reg = 0;
  TC->INTENSET.bit.MC0 = 1;

  NVIC_EnableIRQ(TC3_IRQn);

  TC->CTRLA.reg |= TC_CTRLA_ENABLE;
  while (TC->STATUS.bit.SYNCBUSY == 1);
}

void TC3_Handler() {
  TcCount16* TC = (TcCount16*) TC3;
  // If this interrupt is due to the compare register matching the timer count
  // we run the TimerInterrupt() function.
  if (TC->INTFLAG.bit.MC0 == 1) {
    TC->INTFLAG.bit.MC0 = 1;
    TimerInterrupt();           //THE FUNCTION TO RUN ON THE TIMER
  }
}
