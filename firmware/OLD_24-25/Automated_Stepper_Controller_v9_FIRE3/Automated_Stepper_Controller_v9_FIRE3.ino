// Combustion Chamber pressure based rocket motor tuning using DRV8825 and
// two 24V1A stepper motors (1.8 degrees, 0.4 N*m)
//HIGH - CW Stepper = CCW Valve (Closing)
//LOW - CCW Stepper = CW Valve (Opening)
#include <math.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
//====================================== {Pressure Transducer Prerequisites} ==========================================================================
const int sensorPin = A0;  // Analog pin for pressure transducer
const float sensorMin = 0.5;  // Transducer min Signal voltage Ouput
const float sensorMax = 4.5;    // Transducer max Signal voltage Output
const float pressureMin = 0;   // Minimum transducer PSI  
const float pressureMax = 1000;  // Maximum transducer PSI
const float psi_init = 14.6959; //assuming 1 atm as intial CC pressure
float psi_offset = 0.0; // Initializes offset for calibration
float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) { // Function to map voltage to PSI (floating-point version of map)
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
float readSmoothedPressure(int samples = 10) { //Smooths pressure reading using moving average + CALIBRATION OFFSET
  float sum = 0.0;
  for (int i = 0; i < samples; i++) {
    int sensorValue = analogRead(sensorPin);
    float voltage = sensorValue * (5.0 / 1024.0);
    sum += mapFloat(voltage, sensorMin, sensorMax, pressureMin, pressureMax);
    delay(10);  // Short delay between samples  
  }
  float psi = sum / samples; //Average PSI
  return psi + psi_offset; //Applies calibration offset
}
//==================================== {Farthing's Equations Prerequisites} ============================================================================
float last_T_n = 0.0;
float last_T_e = 0.0;
float Last_T_n_ref = 0.0;
float Last_T_e_ref = 0.0;
#define TURN_THRESHOLD 0.005  // Threshold of 1 step - 1 turn/200steps = 0.005
unsigned long lastUpdate = 0;
const unsigned long updateInterval = 10;  // Update every 750ms <-------------------------------------|
//====================================== {Motor Drivers Prerequisites} =================================================================================
#define DIR_PIN_e 2
#define STEP_PIN_e 3
#define DIR_PIN_n 4
#define STEP_PIN_n 5
int MotorCycle = 400; //Number of Steps to Complete 1 Stepper Motor Cycle
//========================================== {LORA Prerequisites} ======================================================================================
  //Latest version of Meshtastic firmware installed (2.5.20)
  //Dont need to #define since Serial1 is hardwired to pins 0 and 1
  //Wire Tbeam RX --> Arduino TX
  //Wire Tbeam TX --> Arduino RX
bool systemRunning = false; //assigns initial state of systemRunning - used for holding until "START"
//================================== { Data Transmission} ========================================================
unsigned long lastDataTransmission = 0;
const unsigned long dataTransmissionInterval = 50; //Sends Valve Data every 0.5 seconds <------------------| 
// Stores Valve Data outside of loop to prevent overwriting
float global_psi = 0.0;
float global_D_n = 0.0;
float global_D_e = 0.0;
float global_T_n = 0.0;
float global_T_e = 0.0;
float global_delta_T_n = 0.0;
float global_delta_T_e = 0.0;
float global_Last_T_n_ref = 0.0;
float global_Last_T_e_ref = 0.0;
//=================================== {Initalize} ==================================================================
bool systemInitialized = false;
unsigned long lastCommandTime = 0;
const unsigned long debounceDelay = 300;
//=================================== {SD Card} ==================================================
File logFile;
char logFilename [20];
int fileIndex = 0;

//======================================== {Void Setup} ================================================================================================
void setup() {
  // Start Data Log
  Serial.begin(115200);  // Set baud rate to 115200 for stable communication
  Serial1.begin(115200); // Starts LoRa Comm
  //Calibration - Pressure Transducer
  Serial.println("Calibrating Pressure Transducer");
  float initial_voltage = 0.0;
  const int cal_samples = 100; // calibration samples
  for (int i=0; i<cal_samples; i++){
    int sensorValue = analogRead(sensorPin);
    initial_voltage += sensorValue * (5.0/1024.0); //Converts ADC to Voltage
    delay(10);
  }
  initial_voltage /= cal_samples; //takes average of voltages
  float psi_average = mapFloat(initial_voltage, sensorMin, sensorMax, pressureMin, pressureMax);
  psi_offset = psi_init - psi_average; //calculates offset with respect to 1atm = 14.6959
  Serial.println("=======Calibration Complete=======");
  Serial.print("initial_voltage: "); Serial.println(initial_voltage, 3);
  Serial.print("Initial PSI (before calibration): "); Serial.println(psi_average, 2);
  Serial.print("PSI offset: "); Serial.println(psi_offset, 2);
  Serial.print("Calibrate PSI: "); Serial.println(psi_average + psi_offset, 2);
  Serial1.println("Calibration Complete. SYSTEM READY...");


  //SD Card Initialization
  Serial.print("Initalizing SD Card...");
  if (!SD.begin(10)) {
    Serial.println("SD Initialization Failed");
    while(1);
  }
  Serial.println("SD Initialized");
  //File Name Generation
  do {
    snprintf(logFilename, sizeof(logFilename), "log_%03d.csv", fileIndex++);
  } while (SD.exists(logFilename)); //Increments file number until unused name found

  logFile = SD.open(logFilename, FILE_WRITE);
  if(logFile) {
    Serial.print("Logging to: ");
    Serial.println(logFilename);
    logFile.println("Timestamp(ms),Pressure(PSI)");
    logFile.flush(); 
  } else {
    Serial.println("Error Opening Log File");
  }

  //HOLD
  unsigned long lastWaitMessage = 0; 
  const unsigned long waitMessageInterval = 15000; //Wait 15 sec to send message to comms channel
  while (!systemRunning) { //enters loop and prevents void setup continuation
    if (millis() - lastWaitMessage >= waitMessageInterval) { //Sends status message only every waitMessageInterval
    Serial.println("Waiting for START command...");
    Serial1.println("Waiting for START command...");
    lastWaitMessage = millis();
    }
    if (Serial1.available()){ //checks availability of Meshtastic app via LoRa - when message is received loop continue loop
      String line = Serial1.readStringUntil('\n'); //reads incoming until newline is detected
      if (line.startsWith("GSE1: ")) {
        String command = line.substring(6);
        command.trim(); //removes extra spaces or newlines from received command

        unsigned long currentTime = millis();
        if (currentTime - lastCommandTime < debounceDelay) {
          return; //ignores command if there is duplicated messaged sent within debounceDelay
        }
        lastCommandTime = currentTime;
        Serial.print("Received Command: ");
        Serial.println(command);

        if (command.equals("INITIALIZE")) {
          if (!systemInitialized) {
            //Setting Motor Inital Position
            // Initialize motor 1 - Ethanol (e)
            pinMode(DIR_PIN_e, OUTPUT);
            pinMode(STEP_PIN_e, OUTPUT);
            // Initialize motor 2 - Nitrous (n)
            pinMode(DIR_PIN_n, OUTPUT);
            pinMode(STEP_PIN_n, OUTPUT);
            //Intial Motor Position - assuming 1 atm
            float psi_sq_init = psi_init * psi_init;
            //Valve Diameter Calc
            float D_n_init = ((4.12e-6) * psi_sq_init) + ((1.96e-4) * psi_init) + (3.97e0);
            float D_e_init = ((2.35e-6) * psi_sq_init) + ((2.50e-4) * psi_init) + (2.70e0);
            //Valve Turn Calc
            float T_n_init = 0.0076 * (exp(1.66*D_n_init)) + 0.594;
            float T_e_init = 0.0076 * (exp(1.66*D_e_init)) + 0.594;
            //Convert Turns to Steps
            int InitialStep_n = MotorCycle * T_n_init;
            int InitialStep_e = MotorCycle * T_e_init;
            Serial.print("InitialStep_n: "); Serial.println(InitialStep_n);
            Serial.println("D_n_init: "); Serial.println(D_n_init, 3);
            Serial.println("T_n_init: "); Serial.println(T_n_init, 3);
            Serial.println("InitialStep_e: "); Serial.println(InitialStep_e);
            Serial.println("D_e_init: "); Serial.println(D_e_init, 3);
            Serial.println("T_e_init: "); Serial.println(T_e_init, 3);
            Serial.println("Initial Position Calculation Complete");
            delay(3000);
            //Initial Step Nitrous
            for (int i=0; i < InitialStep_n; i++) {
              digitalWrite(DIR_PIN_n,LOW);
              digitalWrite(STEP_PIN_n,HIGH);
              delayMicroseconds(500);
              digitalWrite(STEP_PIN_n,LOW);
              delayMicroseconds(500);
            }
            //Initial Step Ethanol
              for (int i=0; i < InitialStep_e; i++) {
              digitalWrite(DIR_PIN_e,LOW);
              digitalWrite(STEP_PIN_e,HIGH);
              delayMicroseconds(500);
              digitalWrite(STEP_PIN_e,LOW );
              delayMicroseconds(500);
            }
            // Update last_T values to reflect initial turns
            last_T_n = T_n_init;
            last_T_e = T_e_init;
            Serial.println("Motors initialized to Start Position.");
            delay(100);
          } else {
            Serial.println("Already Initalized");
          }
        }

        else if (command.equals("START")) {
          if (!systemInitialized) {
            Serial.println("System has NOT Been Intialized");
          }
          delay(2000); // DELAY FOR NEEDLE VALVES TO RUN AT RAMP-UP VALVE DIAMTER FUNCTIONS =================================================
          systemRunning = true; //changes system state bolean - exits loop & continues to void setup
          Serial.println("Command Received. SYSTEM RUNNING...");
          Serial1.println("Command Received. SYSTEM RUNNING...");
        } else {
          Serial.println("Improper Command");
          Serial1.println("Improper Command");
        }
      }
    }
    delay(100); // Checks Tbeam TX every 100 ms
  }
} //END - Void Setup




//============================================ {Void Loop} =============================================================================================
void loop() {
  //Valve Controls
  if (systemRunning && (millis() - lastUpdate >= updateInterval)) {
    lastUpdate = millis();  // Update time for next read
    float psi = readSmoothedPressure(); //Read Smoothed Pressure

    // Compute valve diameters using corrected quadratic equations
    float psi_sq = psi * psi; // Store psi^2 to avoid recalculating
    float D_n = ((4.12e-6) * psi_sq) + ((1.96e-4) * psi) + (3.97e0);
    float D_e = ((2.35e-6) * psi_sq) + ((2.50e-4) * psi) + (2.70e0);
    // Compute number of turns using the new exponential fit function
    float T_n = 0.0076 * (exp(1.66*D_n)) + 0.594;
    float T_e = 0.0076 * (exp(1.66*D_e)) + 0.594;
    // Compute the difference in turns (ΔT)
    float delta_T_n = T_n - last_T_n; //% turns
    float delta_T_e = T_e - last_T_e; //% turns
    //Store Valve Data to Global Variables
    global_psi = psi;
    global_D_n = D_n;
    global_D_e = D_e;
    global_T_n = T_n;
    global_T_e = T_e;
    global_delta_T_n = delta_T_n;
    global_delta_T_e = delta_T_e;
    global_Last_T_n_ref = last_T_n;
    global_Last_T_e_ref = last_T_e;
    // Update last turns
    last_T_n = T_n;
    last_T_e = T_e;
    //Direction Control
    if (delta_T_n > TURN_THRESHOLD){ //if positive turn value
      digitalWrite(DIR_PIN_n,LOW); //turns off Dir Pin allowing for CCW turn
    } else if (delta_T_n < -TURN_THRESHOLD) {
      digitalWrite(DIR_PIN_n, HIGH); //CW
    }
    if (delta_T_e > TURN_THRESHOLD) {
     digitalWrite(DIR_PIN_e, LOW); // CCW turn
      Serial.println("Ethanol Motor: Turning CCW (Opening)");
    } else if (delta_T_e < -TURN_THRESHOLD) {
     digitalWrite(DIR_PIN_e, HIGH); // CW turn
      Serial.println("Ethanol Motor: Turning CW (Closing)");
    } else {
     Serial.println("Ethanol Motor: No movement");
    }
    //Step Control
    int StepsToMove_n = round(MotorCycle * delta_T_n); //number of steps with respect to % turn
    int StepsToMove_e = round(MotorCycle * delta_T_e); //rounds to avoid skipping small moves
    unsigned int stepDelay_n = 500;
    unsigned int stepDelay_e = 500;
    if (abs(delta_T_n) > TURN_THRESHOLD && StepsToMove_n !=0) { //if delta_T is  larger then 0.01 steps (1% motor turn) && loop doesnt start if ~=0
      for (int i=0; i < abs(StepsToMove_n); i++) { //abs in the event delta_T produces (-). for loops cant comp (-)
        digitalWrite(STEP_PIN_n, HIGH);
        delayMicroseconds(stepDelay_n);
        digitalWrite(STEP_PIN_n, LOW);
        delayMicroseconds(stepDelay_n);
      }
    }
    if (abs(delta_T_e) > TURN_THRESHOLD && StepsToMove_e !=0) { //if delta_T is  larger then 0.01 steps (1% motor turn) && loop doesnt start if ~=0
      for (int i=0; i < abs(StepsToMove_e); i++){
        digitalWrite(STEP_PIN_e, HIGH);
        delayMicroseconds(stepDelay_e);
        digitalWrite(STEP_PIN_e, LOW);
        delayMicroseconds(stepDelay_e);
      }
    }
    //writes to log file
    if (logFile) {
      logFile.print(millis());
      logFile.print(",");
      logFile.println(global_psi, 2);
      logFile.flush(); 
    }
  } //END - Valve Control

  //closes log file
  static bool fileClosed = false;
  if (!systemRunning && logFile && !fileClosed) {
    logFile.close();
    fileClosed = true;
  }
  if (systemRunning) {
    fileClosed = false;
  }

  //Data Tranmission Interval
  if ((millis() - lastDataTransmission >= dataTransmissionInterval)) {
    // Display results in Arduino Serial Monitor
    Serial.println("=====================================");
    Serial.print("PSI: "); Serial.print(global_psi, 4); Serial.println(" psi");
    Serial.print("Nitrous Valve Diameter: "); Serial.print(global_D_n, 3); Serial.println(" mm");
    Serial.print("Ethanol Valve Diameter: "); Serial.print(global_D_e, 3); Serial.println(" mm");
    Serial.print("Nitrous Valve Turn Change: "); Serial.print(global_delta_T_n, 3); Serial.println(" turns");
    Serial.print("Ethanol Valve Turn Change: "); Serial.print(global_delta_T_e, 3); Serial.println(" turns");
    Serial.println("=====================================\n");

    lastDataTransmission = millis();
  }
}