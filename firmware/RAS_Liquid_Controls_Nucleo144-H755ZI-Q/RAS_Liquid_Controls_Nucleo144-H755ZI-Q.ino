// SD CARD
#include <SPI.h>
#include <SD.h>
File dataFile; // define variable for the data file
int dataFileNum = 1; // define variable necessary to ensure files are not overwritten (see SD card setup below)
const int chipSelect = 26; // CN10 pin 13 -> D26
Sd2Card card; // not needed?
SdVolume volume; // not needed?
SdFile root; //not needed?

bool debug = true;

void setup() {
  // put your setup code here, to run once:

    // SERIAL COMMUNICATION SETUP
  Serial.begin(9600); //start debug serial channel
  if (debug) {Serial.println("\nSerial initialized");}

  // SD CARD SETUP
  if (!card.init(SPI_HALF_SPEED, chipSelect) && debug) {Serial.println("SD card initialization failed");}
  if (!volume.init(card)) {Serial.println("SD card formatting is incorrect");}

  SD.begin(chipSelect); // create SD card variable
  dataFileNum = 1;
  while (SD.exists("data" + String(dataFileNum) + ".csv")) { // while loop checks for other files that have already been written to avoid overwriting anything
    dataFileNum = dataFileNum + 1;
  }
  dataFile = SD.open("data" + String(dataFileNum) + ".csv", FILE_WRITE);
  dataFile.println("FILE HEADER HERE"); dataFile.println(""); // write a file header
  dataFile.println("Time (ms),Force (lb),Pressure (psi)"); // write the variable names
  dataFile.close(); // always close the file, otherwise it will not save properly

}

void loop() {
  // put your main code here, to run repeatedly:

  //print data to SD card:
  dataFile = SD.open("data" + String(dataFileNum) + ".csv", FILE_WRITE);
  dataFile.println("TEST LINE");
  dataFile.close(); // always close the file, otherwise it will not save properly

}
