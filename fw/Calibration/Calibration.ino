//Firmware for MiniTruss remote lab experiment
// Single controller firmware with state machine for reading and writing.

//Author: Timothy D. Drysdale
// timothy.d.drysdale@gmail.com
// 05/09/2022

// IMPORT LIBRARIES
#include "ArduinoJson-v6.9.1.h"
#include <FlashStorage.h>

//JSON serialization
#define COMMAND_SIZE 64  //originally 64
StaticJsonDocument<COMMAND_SIZE> doc;
char command[COMMAND_SIZE];

#define FLASH_WRITES_MAX 20
#define SECRET_LEN_MAX 99
#define SCALE_FACTOR_LEN 7

// calibration value indices in cal array
#define LOAD_CELL 0
#define MEMBER_1 1
#define MEMBER_2 2
#define MEMBER_3 3
#define MEMBER_4 4
#define MEMBER_5 6
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


// DELETE
// put cal values in array and use the #define'd indices above to make cal set/get code safer
// volatile float scale_factor[7]; // our working copy of the calibration array so that code does not need to know about flashstorage object

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


  //Serial communication for sending data -> RPi -> Server
  Serial.begin(57600);
  while(!Serial);



}

void loop() {
    Sm_Run();
    report();
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

      
      const char* values = doc["to"];

     
        
    }  
    else if(strcmp(set, "report")==0)
    {
    
      report();
      
    }  
    
    
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
 
