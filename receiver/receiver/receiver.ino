#include <SD.h>
#include <SoftwareSerial.h>
#include <LiquidCrystal_I2C.h>
// #include <AltSoftSerial.h>

// AltSoftSerial bt;
// softserial
#define rxPin 2
#define txPin 3


#define PIN_SPI_CS 10
#define nServoStates1 2
#define nServoStates2 2
#define nServoStates3 3
#define nActions 6
SoftwareSerial bt =  SoftwareSerial(rxPin, txPin);

File file;
LiquidCrystal_I2C lcd(0x27, 16, 2);
void setup() {
    Serial.begin(38400);
    bt.begin(9600);

    // bt.begin(9600);
    if (!SD.begin(PIN_SPI_CS)) {
        // lcd.println(F("SD CARD FAILED, OR NOT PRESENT!"));
        Serial.println("Sd noooo");
    }

    lcd.begin();

    // // Turn on the blacklight
    lcd.setBacklight((uint8_t)1);

    // // First row
    lcd.print("Hello, world!");

}

void writeQToSD(){
    file = SD.open("QTable.txt", FILE_WRITE | O_TRUNC);
    lcd.clear();
    lcd.println(F("Trying"));
    delay(1000);
    float val;
    if (file) {
      lcd.clear();
      lcd.println("file ok");
      delay(1000);
      while(Serial.available()==0) ;
      val = Serial.parseFloat(SKIP_ALL, '\n');
      lcd.clear();
      lcd.print(val);
      file.print(val);
      file.write("\n");
      delay(50);
      for(int i = 0; i < nServoStates1; i++)
          for(int j = 0; j < nServoStates2; j++)
              for(int k = 0; k < nServoStates3; k++)
                  for(int l = 0; l < nActions; l++){
                      while(Serial.available()==0) ;
                      val = Serial.parseFloat(SKIP_ALL, '\n');
                      lcd.clear();
                      lcd.print(val);
                      file.print(val);
                      file.write(' ');
                      while(Serial.available()==0) ;
                      val = Serial.parseFloat(SKIP_ALL, '\n');
                      lcd.clear();
                      lcd.print(val);
                      file.print(val);
                      file.write(' ');
                      while(Serial.available()==0) ;
                      val = Serial.parseFloat(SKIP_ALL, '\n');
                      file.print(val);
                      file.write("\n");
                      lcd.clear();
                      lcd.print("Ok");
                      delay(50);
                  }
      file.close();
      lcd.print("DONE");
      delay(10000);
    } else {
        Serial.print(F("SD Card: error on opening file arduino.txt"));
    }              
}

void readAndSd(){
   file = SD.open("QTABLE.txt");
   lcd.clear();
   lcd.print("here");
   delay(3000);
  float val;
    if (file) {
        lcd.print("FIle found");
        Serial.print("receive");
        delay(3000);
        lcd.clear();
        val = file.parseFloat();
        Serial.println(val);
        lcd.print(val);
        delay(250);
        lcd.clear();
        for(int i = 0; i < nServoStates1*nServoStates2*nServoStates3; i++){
          for(int j = 0; j < nActions; j++){
             val = file.parseFloat();
             Serial.println(val);
             lcd.print(val);
             delay(250);
             lcd.clear();
            //  Serial.print(" ");
             val = file.parseFloat();
             Serial.println(val);
             lcd.print(val);
             delay(250);
             lcd.clear();
             val = file.parseFloat();
             Serial.println(val);
             lcd.print(val);
             delay(250);
             lcd.clear();
            //  Serial.println();
          }
        }
        file.close();
        lcd.print("DONNEEEE");
        delay(4000);
    } else {
        Serial.print(F("SD Card: error on opening file arduino.txt"));
    } 
}
bool done = false;
// the loop function runs over and over again forever
void loop() {
  if(bt.available()){
    String cmd = bt.readString();
    lcd.clear();
    lcd.print(cmd);
    if(cmd == "load"){
      readAndSd();
    }
    if(cmd == "save"){
      Serial.print("send"); 
      lcd.print("Ready"); 
      delay(1000);
      writeQToSD();
    }
  }
  if(Serial.available()){
    float value = Serial.parseFloat();
    lcd.clear();
    lcd.print("Progress");
    lcd.setCursor(0, 1);
    lcd.print(value);
  }
  
}
