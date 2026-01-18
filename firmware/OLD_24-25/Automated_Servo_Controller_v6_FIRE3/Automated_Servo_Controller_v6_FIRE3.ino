//The Servo Shield uses I2C to communicate, 2 pins are required to  
//interface. For Arduino UNOs, thats SCL -> Analog 5, SDA -> Analog 4

#include <Wire.h> //enables I2C comms on Arduino
#include <Adafruit_PWMServoDriver.h> //Controls PWM chip

//========================================== {LORA Prerequisites} ======================================================================================
  //Latest version of Meshtastic firmware installed (2.5.20)
  //Dont need to #define since Serial1 is hardwired to pins 0 and 1
  //Wire Tbeam RX --> Arduino TX - CONSIDER REMOVING TO MINIMIZE SIGNAL CLOG
  //Wire Tbeam TX --> Arduino RX
bool systemRunning = false; //assigns initial state of systemRunning - used for holding until "START"
bool systemInitialized = false; // used for initialized loop
unsigned long lastCommandTime = 0; 
const unsigned long debounceDelay = 300; //milli seconds delay for "INITIALIZE" command 
//========================================
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(); //Creates PWM object
uint16_t angleToPulse(int angle) {
  angle = constrain(angle, 0, 180); //limits servo movement between 0 and 90 degrees
  return map(angle, 0, 180, 512, 2560); //maps 0 to 90 degrees to the 150 to 600 pulse width of se
}
uint16_t angleToPulse2(int angle) {
  angle = constrain(angle, 0, 270);
  return map(angle, 0, 270, 512, 2560); 
}
//Relay Prerequisites
int relayPin = 7; //Relay Pin "IN"


void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);
  pwm.begin();
  pwm.setPWMFreq(250); //Servo Support Frequence: 50-333Hz
  pinMode(relayPin, OUTPUT);
  digitalWrite(relayPin, LOW);
  delay(5000); //Manual hold to allow time for needle valves to open

  //Phase 0: All Valves Closed
  pwm.setPWM(1, 0, angleToPulse(180)); //Reversed
  pwm.setPWM(2, 0, angleToPulse(0)); 
  pwm.setPWM(3, 0, angleToPulse(180)); //Reversed
  pwm.setPWM(4, 0, angleToPulse2(0));
  pwm.setPWM(5, 0, angleToPulse(180)); //Reversed
  pwm.setPWM(6, 0, angleToPulse(0));
  pwm.setPWM(7, 0, angleToPulse(0));
  pwm.setPWM(8, 0, angleToPulse(0));
  pwm.setPWM(9, 0, angleToPulse(0));
  delay(100);

    //HOLD
  unsigned long lastWaitMessage = 0; 
  const unsigned long waitMessageInterval = 15000; //Wait 15 sec to send message to comms channel
  while (!systemRunning) { //enters loop and prevents void setup continuation
    if (millis() - lastWaitMessage >= waitMessageInterval) { //Sends status message only every waitMessageInterval
    Serial.println("All Valves Closed; Waiting for START command...");
    lastWaitMessage = millis();
    }
    if (Serial1.available()){ //checks availability of Meshtastic app via LoRa - when message is received loop continue loop
      String line = Serial1.readStringUntil('\n'); //reads incoming until newline is detected
      if (line.startsWith("GSE1: ")) { //GSE1
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
            //Phase 1: Open Hold Position Before Start
              //Open Valves: B5, B6, B1, B9
            pwm.setPWM(5, 0, angleToPulse(90));
            pwm.setPWM(6, 0, angleToPulse(90));
            pwm.setPWM(1, 0, angleToPulse(90));
            pwm.setPWM(9, 0, angleToPulse(90));
            Serial.println("Phase 2 Open - HOLD");
            delay(5000); //allow enough time for Nitrous to settle to liquid
            systemInitialized = true;
          } else {
            Serial.println("Already Initialized");
          }
        }
        
        else if (command.equals("START")) {
          if (!systemInitialized) {
            Serial.println("System Has NOT Been Initialized");
          } else {
            systemRunning = true; //changes system state bolean - exits loop & continues to void setup
            Serial.println("Start Command Received. SYSTEM RUNNING");

            //delay(5000); // TIMING OFFSET TO INIATE STEPPER FIRST - DONT THINK I NEED THIS ANYMORE FOR CORRECT TIMING

            //Phase 2: START
              //Open Valves: B7 & B8
            pwm.setPWM(7, 0, angleToPulse(90));
            pwm.setPWM(8, 0, angleToPulse(90));
            Serial.println("Phase 3 Open - Main Ball Valve");
            delay(300); //================================================
            digitalWrite(relayPin, HIGH);
            Serial.println("Ignition On");
            delay(3000); //FULL BURN
              //Close Valves: B5, B6, B9, B1
            pwm.setPWM(5, 0, angleToPulse(180)); //Reversed
            pwm.setPWM(6, 0, angleToPulse(0));
            pwm.setPWM(9, 0, angleToPulse(0));
            pwm.setPWM(1, 0, angleToPulse(180)); //Reversed
            Serial.println("Phase 3 Close - Main Ball Valve");
            digitalWrite(relayPin, LOW);
            Serial.println("Ignition OFF");
            delay(2000);


            //Phase 3: Nitro & Eth Purge
              //Open Valves: B2, B4, B3
            pwm.setPWM(2, 0, angleToPulse(90));
            pwm.setPWM(4, 0, angleToPulse2(90));
            pwm.setPWM(3, 0, angleToPulse(90));
            Serial.println("Phase 4 Open - Nitro Purge");
            delay(500);
              //Close Valves: B3, B2, B4, B7, B8
            pwm.setPWM(3, 0, angleToPulse(180)); //Reversed
            pwm.setPWM(2, 0, angleToPulse(0));
            pwm.setPWM(4, 0, angleToPulse2(0)); 
            pwm.setPWM(7, 0, angleToPulse(0));
            pwm.setPWM(8, 0, angleToPulse(0));
            Serial.println("Phase 4 Close - Nitro Purge");

          }
        
        }

        else if (command.equals("DEPRESSURIZE")) {
          if (!systemInitialized) {
            Serial.println("System Has NOT Been Initialized");
          } else {
            pwm.setPWM(7, 0, angleToPulse(90));
            pwm.setPWM(8, 0, angleToPulse(90)); 
          }
          
        } else {
          Serial.println("Improper Command");
        }
      }
    }
    delay(100); // Checks Tbeam TX eveery 100 ms
  }
}

void loop() {

}
