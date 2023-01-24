#include<Servo.h>
#include<NewPing.h>
#include <Wire.h>
#include <SD.h>

#define PIN_SPI_CS 4
// sd card manage 
File file;


#include <Adafruit_PWMServoDriver.h>
// Servo 1 = Nicher ta connected at 0 
// Servo 2 = Middle connected at 4
// Servo 3 = top connected at 8


#define SERVO1_PIN 0
#define SERVO2_PIN 4
#define SERVO3_PIN 8
#define SERVOMIN  125 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  575 // this is the 'maximum' pulse length count (out of 4096)
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Sonar
// sonar 1 = at the back, used for distance forwarded or not
// sonar 2, 3 at the forward corner. User to know whether there's something forward
#define TRIG_PIN1 4
#define ECHO_PIN1 5
#define TRIG_PIN2 6
#define ECHO_PIN2 7
#define TRIG_PIN3 8
#define ECHO_PIN3 9
#define MAX_DISTANCE 300
NewPing sonar1(TRIG_PIN1, ECHO_PIN1, MAX_DISTANCE); 
NewPing sonar2(TRIG_PIN2, ECHO_PIN2, MAX_DISTANCE);
NewPing sonar3(TRIG_PIN3, ECHO_PIN3, MAX_DISTANCE);


// algo var
float alpha = 0.75;
float gamma = 0.1;
float epsilon;

const int nServoStates1 = 4;
float minServoAngle1 = 40;
float maxServoAngle1 = 60;
float initialServoAngle1 = 50;
float deltaAngle1 = (maxServoAngle1 - minServoAngle1) / (nServoStates1 - 1);
int state1 = int((initialServoAngle1 - minServoAngle1)/deltaAngle1);
float delayTime1 = 4.5*deltaAngle1;

const int nServoStates2 = 4;
float minServoAngle2 = 90;
float maxServoAngle2 = 110;
float initialServoAngle2 = 100;
float deltaAngle2 = (maxServoAngle2 - minServoAngle2) / (nServoStates2 - 1);
int state2 = int((initialServoAngle2 - minServoAngle2)/deltaAngle2);
float delayTime2 = 4.5*deltaAngle2;

const int nServoStates3 = 4;
float minServoAngle3 = 40;
float maxServoAngle3 = 60;
float initialServoAngle3 = 50;
float deltaAngle3 = (maxServoAngle3 - minServoAngle3) / (nServoStates3 - 1);
int state3 = int((initialServoAngle3 - minServoAngle3)/deltaAngle3);
float delayTime3 = 4.5*deltaAngle3;

const int nActions = 6;
float servoAngles[3] = {initialServoAngle1, initialServoAngle2, initialServoAngle3};

float Q[nServoStates1][nServoStates2][nServoStates3][nActions];
// float Q[3][3][3][6];

int reward = 0;
int currentDistance = 0;
int previousDistance = 0;
int deltaDistance = 0;
float lookahead = 0;
float sample = 0;
int action = 0;

void setup(){
    
    delay(5000);
    Serial.begin(9600);
    pwm.begin();
  
    pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates
    pwm.setPWM(SERVO1_PIN, 0, angleToPulse(initialServoAngle1));
    pwm.setPWM(SERVO2_PIN, 0, angleToPulse(initialServoAngle2));
    pwm.setPWM(SERVO3_PIN, 0, angleToPulse(initialServoAngle3));
    yield();

    if (!SD.begin(PIN_SPI_CS)) {
        Serial.println(F("SD CARD FAILED, OR NOT PRESENT!"));
    }
}

// returns pulse for angle
int angleToPulse(int ang){
   int pulse = map(ang,0, 180, SERVOMIN,SERVOMAX);// map angle of 0 to 180 to Servo min and Servo max 
  //  Serial.print("Angle: ");Serial.print(ang);
  //  Serial.print(" pulse: ");Serial.println(pulse);
   return pulse;
}

int getAction(){
    int isActionValid[] = {-1, -1, -1, -1, -1, -1};
    int bestValue = -1000000;
    int bestAction;
    int action;

    if(state1 + 1 != nServoStates1){
        isActionValid[0] = 1;
        if(Q[state1][state2][state3][0] > bestValue){
            bestValue = Q[state1][state2][state3][0];
            bestAction = 0;
        }
    }
    else if(state1 != 0){
        isActionValid[1] = 1;
        if(Q[state1][state2][state3][1] > bestValue){
            bestValue = Q[state1][state2][state3][1];
            bestAction = 1;
        }
    }
    else if(state2 + 1 != nServoStates2){
        isActionValid[2] = 1;
        if(Q[state1][state2][state3][2] > bestValue){
            bestValue = Q[state1][state2][state3][2];
            bestAction = 2;
        }
    }
    else if(state2 != 0){
        isActionValid[3] = 1;
        if(Q[state1][state2][state3][3] > bestValue){
            bestValue = Q[state1][state2][state3][3];
            bestAction = 3;
        }
    }
    else if(state3 + 1 != nServoStates3){
        isActionValid[4] = 1;
        if(Q[state1][state2][state3][4] > bestValue){
            bestValue = Q[state1][state2][state3][4];
            bestAction = 4;
        }
    }
    else if(state3 != 0){
        isActionValid[3] = 1;
        if(Q[state1][state2][state3][5] > bestValue){
            bestValue = Q[state1][state2][state3][5];
            bestAction = 5;
        }
    }


    float randomValue = random(0, 101);
    if(randomValue < (1 - epsilon)*100){
        // take best action
        action = bestAction;
    }
    else{
        // explore
        bool randomActionFound = false;
        while(!randomActionFound){
            action = random(0, nActions);
            if(isActionValid[action] == 1){
                randomActionFound = true;
            }
        }
    }
    return action;
}

void goToNextState(int action){
    // for an action, only one state will change
    // for example rotating servo1 ccw reduces state
    if(action == 0){
        state1++;
    }
    else if(action == 1){
        state1--;
    }
    else if(action == 2){
        state2++;
    }
    else if(action == 3){
        state2--;
    }
    else if(action == 4){
        state3++;
    }
    else if(action == 5){
        state3--;
    }
}

void executeAction(int action){
    // executes action i.e rotate a servo cw\ccw

    if(action == 0){
        servoAngles[0] = servoAngles[0] + deltaAngle1;
        pwm.setPWM(SERVO1_PIN, 0, angleToPulse(servoAngles[0]));
        delay(delayTime1);
    }
    else if(action == 1){
        servoAngles[0] = servoAngles[0] - deltaAngle1;
        pwm.setPWM(SERVO1_PIN, 0, angleToPulse(servoAngles[0]));
        delay(delayTime1);
    }
    else if(action == 2){
        servoAngles[1] = servoAngles[1] + deltaAngle2;
        pwm.setPWM(SERVO2_PIN, 0, angleToPulse(servoAngles[1]));
        delay(delayTime2);

    }
    else if(action == 3){
        servoAngles[1] = servoAngles[1] - deltaAngle2;
        pwm.setPWM(SERVO2_PIN, 0, angleToPulse(servoAngles[1]));
        delay(delayTime2);
    }
    else if(action == 4){
        servoAngles[2] = servoAngles[2] + deltaAngle3;
        pwm.setPWM(SERVO3_PIN, 0, angleToPulse(servoAngles[2]));
        delay(delayTime3);
    }
    else if(action == 5){
        servoAngles[2] = servoAngles[2] - deltaAngle3;
        pwm.setPWM(SERVO3_PIN, 0, angleToPulse(servoAngles[2]));
        delay(delayTime3);
    }
}
float previousShift = 0;

float getDistance(){
    // reward function
    // rewards forward movement (when sonar1 reports more values than before)
    // penlizes keeping obstacle in front (when sonar2 and sonar3 reports small values which is not zero,
    // as zero might mean there is no object in the range)
    // shift helps us understand if we are moving in the right direction
    // towards avoiding the obstacle
    currentDistance = sonar1.ping_cm();
    deltaDistance = currentDistance - previousDistance;
    
    if(deltaDistance > 100 || deltaDistance < -50) deltaDistance = 0;
    previousDistance = currentDistance;
    Serial.println(currentDistance);
    float penalty = 0;
    float s2Dist = sonar2.ping_cm(), s3Dist = sonar3.ping_cm();
    if(s2Dist > 3 && s2Dist < 20 || s3Dist > 3 && s3Dist){
        // zero distance means it is good
        if(s2Dist == 0 || s2Dist > 25) s2Dist = 25;
        if(s3Dist == 0 || s3Dist > 25) s3Dist = 25;
        penalty += (25 - s2Dist) / (17*2);
        penalty += (25 - s3Dist) / (17*2);
        float currentShift = abs(s2Dist - s3Dist);
        penalty -= (currentShift - previousShift);
        previousShift = currentShift;
    }
    else{
        previousShift = 0;
    }
    deltaDistance -= penalty;
    return deltaDistance;
}

float getLookahead(){
    float bestValue = -1000000;
    if(state1 + 1 != nServoStates1){
        bestValue = max(Q[state1][state2][state3][0], bestValue);
    }
    else if(state1 != 0){
        bestValue = max(bestValue, Q[state1][state2][state3][1]);
    }
    else if(state2 + 1 != nServoStates2){
        bestValue = max(bestValue, Q[state1][state2][state3][2]);
    }
    else if(state2 != 0){
        bestValue = max(bestValue, Q[state1][state2][state3][3]);
    }
    else if(state3 + 1 != nServoStates3){
        bestValue = max(bestValue, Q[state1][state2][state3][4]);
    }
    else if(state3 != 0){
        bestValue = max(bestValue, Q[state1][state2][state3][5]);
    }

    return bestValue;
}


void initializeQ(){
    for(int i = 0; i < nServoStates1; i++)
        for(int j = 0; j < nServoStates2; j++)
            for(int k = 0; k < nServoStates3; k++)
                for(int l = 0; l < nActions; l++)
                    Q[i][j][k][l] = 10.0;
}

void wrtiteQToSD(){
    file = SD.open("QTable.txt", FILE_WRITE);

    if (file) {
        for(int i = 0; i < nServoStates1; i++)
            for(int j = 0; j < nServoStates2; j++)
                for(int k = 0; k < nServoStates3; k++)
                    for(int l = 0; l < nActions; l++){
                        file.write(Q[i][j][k][l]);
                        file.write(' ');
                    }
        file.close();
    } else {
        Serial.print(F("SD Card: error on opening file arduino.txt"));
    }              
}


void readQFromSD(){
    file = SD.open("QTable.txt");

    if (file) {
        for(int i = 0; i < nServoStates1; i++)
            for(int j = 0; j < nServoStates2; j++)
                for(int k = 0; k < nServoStates3; k++)
                    for(int l = 0; l < nActions; l++){
                        Q[i][j][k][l] = file.parseFloat();
                    }
        file.close();
    } else {
        Serial.print(F("SD Card: error on opening file arduino.txt"));
    }              
}

// TO-DO : do bluetooth thing, then we are done
void checkBTCommands(){
  int command = 0;
  if (Serial.available()) {
    command = Serial.read();
  }
  if (command == 's') {
    wrtiteQToSD();
  } else if (command == 'l') {
    readQFromSD();
  } else if (command == 'c') {

  }
//   if(command != 0) printAngles();
  
}

int tt = 0; 
void loop(){
    tt++;
    epsilon = exp(-tt/1000);
    action = getAction();
    goToNextState(action);
    executeAction(action);
    reward = getDistance();
    lookahead = getLookahead();
    sample = reward + gamma * lookahead;
    Q[state1][state2][state3][action] += alpha*(sample - Q[state1][state2][state3][action]);
    if(tt==1)initializeQ();
    // Serial.println(action);
    Serial.println(state2);
    Serial.println(state1);
    delay(1000);
}