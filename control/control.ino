#include<NewPing.h>
// #include <SD.h>

#define PIN_SPI_CS 10
// sd card manage 



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
#define TRIG_PIN3 2
#define ECHO_PIN3 3
#define MAX_DISTANCE 300
NewPing sonar1(TRIG_PIN1, ECHO_PIN1, MAX_DISTANCE); 
NewPing sonar2(TRIG_PIN2, ECHO_PIN2, MAX_DISTANCE);
NewPing sonar3(TRIG_PIN3, ECHO_PIN3, MAX_DISTANCE);


// algo var
const float alpha = 0.1; 
const float gamma = 0.75;
float epsilon;

#define nServoStates1 2
#define minServoAngle1 100
#define maxServoAngle1 130
#define initialServoAngle1 130
float deltaAngle1 = (maxServoAngle1 - minServoAngle1) / (nServoStates1 - 1);
int state1 = int((initialServoAngle1 - minServoAngle1)/deltaAngle1);


#define nServoStates2 2
float minServoAngle2 = 45;
float maxServoAngle2 = 55;
float initialServoAngle2 = 45;
float deltaAngle2 = (maxServoAngle2 - minServoAngle2) / (nServoStates2 - 1);
int state2 = int((initialServoAngle2 - minServoAngle2)/deltaAngle2);


const int nServoStates3 = 3;
float minServoAngle3 = 40;
float maxServoAngle3 = 70;
float initialServoAngle3 = 70;
float deltaAngle3 = (maxServoAngle3 - minServoAngle3) / (nServoStates3 - 1);
int state3 = int((initialServoAngle3 - minServoAngle3)/deltaAngle3);


float servoAngles[3] = {initialServoAngle1, initialServoAngle2, initialServoAngle3};
const int nCollisionStates = 2;
const int nStates = nServoStates1*nServoStates2*nServoStates3;
const int nActions = 6;
float Q[nStates][nActions];
float C[nStates][nActions];
float C_[nStates][nActions];
int s = int(state1*nServoStates2*nServoStates3 + state2*nServoStates3 + state3);
int sPrime = s;

// 0 is no,   1 is good left, 2 is good right
int isCollision = 0;
float reward = 0.0;
float currentDistance = 0.0;
float previousDistance = 0.0;
float deltaDistance = 0.0;

float prevD2 = 0.0, prevD3 = 0.0, curD2 = 0.0, curD3 = 0.0;
float lookahead = 0.0;
float sample = 0.0;
int action = 0;

void setup(){
    Serial.begin(38400);
    pwm.begin();

    pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates
    pwm.setPWM(SERVO1_PIN, 0, angleToPulse(initialServoAngle1));
    pwm.setPWM(SERVO2_PIN, 0, angleToPulse(initialServoAngle2));
    pwm.setPWM(SERVO3_PIN, 0, angleToPulse(initialServoAngle3));
    yield();
    delay(2000);    
}

// returns pulse for angle
int angleToPulse(int ang){
   int pulse = map(ang,0, 180, SERVOMIN,SERVOMAX);// map angle of 0 to 180 to Servo min and Servo max 
   return pulse;
}
int tt = 0; 

int getState(int s1, int s2, int s3){
  return s1*nServoStates2*nServoStates3 + s2*nServoStates3 + s3;
}
int getAction(){
    int isActionValid[] = {-1, -1, -1, -1, -1, -1};
    float bestValue = -1000000.0;
    int bestAction;
    int action;
    if(state1 + 1 != nServoStates1){
        isActionValid[0] = 1;
        if(Q[s][0] >= bestValue){
            bestValue = Q[s][0];
            bestAction = 0;
        }
    }
    if(state1 != 0 && isCollision){
        isActionValid[1] = 1;
        if(Q[s][1] >= bestValue){
            bestValue = Q[s][1];
            bestAction = 1;
        }
    }
    if(state2 + 1 != nServoStates2){
        isActionValid[2] = 1;
        if(Q[s][2] >= bestValue){
            bestValue = Q[s][2];
            bestAction = 2;
        }
    }
    if(state2 != 0){
        isActionValid[3] = 1;
        if(Q[s][3] >= bestValue){
            bestValue = Q[s][3];
            bestAction = 3;
        }
    }
    if(state3 + 1 != nServoStates3){
        isActionValid[4] = 1;
        if(Q[s][4] >= bestValue){
            bestValue = Q[s][4];
            bestAction = 4;
        }
    }
    if(state3 != 0){
        isActionValid[5] = 1;
        if(Q[s][5] >= bestValue){
            bestValue = Q[s][5];
            bestAction = 5;
        }
    }

    float randomValue = random(0, 101);
    // Serial.print(F("Progress "));
    if(tt % 20 == 0) Serial.print(2*(1 - epsilon)*100);
    if(randomValue < 2*(1 - epsilon)*100){
        // take best action
        action = bestAction;
    }
    else{
        // explore
        // Serial.println(F("exploring"));
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
int getAction2(){
    int isActionValid[] = {-1, -1, -1, -1, -1, -1};
    float bestValue = -1000000.0;
    int bestAction;
    int action;
    if(state1 + 1 != nServoStates1){
        isActionValid[0] = 1;
        if(C[s][0] >= bestValue){
            bestValue = C[s][0];
            bestAction = 0;
        }
    }
    if(state1 != 0 && isCollision){
        isActionValid[1] = 1;
        if(C[s][1] >= bestValue){
            bestValue = C[s][1];
            bestAction = 1;
        }
    }
    if(state2 + 1 != nServoStates2){
        isActionValid[2] = 1;
        if(C[s][2] >= bestValue){
            bestValue = C[s][2];
            bestAction = 2;
        }
    }
    if(state2 != 0){
        isActionValid[3] = 1;
        if(C[s][3] >= bestValue){
            bestValue = C[s][3];
            bestAction = 3;
        }
    }
    if(state3 + 1 != nServoStates3){
        isActionValid[4] = 1;
        if(C[s][4] >= bestValue){
            bestValue = C[s][4];
            bestAction = 4;
        }
    }
    if(state3 != 0){
        isActionValid[5] = 1;
        if(C[s][5] >= bestValue){
            bestValue = C[s][5];
            bestAction = 5;
        }
    }

    float randomValue = random(0, 101);
    // Serial.print(F("Progress ")); Serial.println(2*(1 - epsilon)*100);
    if(randomValue < 2*(1 - epsilon)*100){
        // take best action
        action = bestAction;
    }
    else{
        // explore
        // Serial.println(F("exploring"));
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
int getAction3(){
    int isActionValid[] = {-1, -1, -1, -1, -1, -1};
    float bestValue = -1000000.0;
    int bestAction;
    int action;
    if(state1 + 1 != nServoStates1){
        isActionValid[0] = 1;
        if(C_[s][0] >= bestValue){
            bestValue = C_[s][0];
            bestAction = 0;
        }
    }
    if(state1 != 0 && isCollision){
        isActionValid[1] = 1;
        if(C_[s][1] >= bestValue){
            bestValue = C_[s][1];
            bestAction = 1;
        }
    }
    if(state2 + 1 != nServoStates2){
        isActionValid[2] = 1;
        if(C_[s][2] >= bestValue){
            bestValue = C_[s][2];
            bestAction = 2;
        }
    }
    if(state2 != 0){
        isActionValid[3] = 1;
        if(C_[s][3] >= bestValue){
            bestValue = C_[s][3];
            bestAction = 3;
        }
    }
    if(state3 + 1 != nServoStates3){
        isActionValid[4] = 1;
        if(C_[s][4] >= bestValue){
            bestValue = C_[s][4];
            bestAction = 4;
        }
    }
    if(state3 != 0){
        isActionValid[5] = 1;
        if(C_[s][5] >= bestValue){
            bestValue = C_[s][5];
            bestAction = 5;
        }
    }

    float randomValue = random(0, 101);
    if(randomValue < 2*(1 - epsilon)*100){
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
    sPrime = int(state1*nServoStates2*nServoStates3 + state2*nServoStates3 + state3);
}

void executeAction(int action){
    // executes action i.e rotate a servo cw\ccw   
    int oldAngle; 
    if(action == 0){
        oldAngle  = servoAngles[0];
        servoAngles[0] = servoAngles[0] + deltaAngle1;
        incrementAngle(SERVO1_PIN, oldAngle, servoAngles[0]);
    }
    else if(action == 1){
        oldAngle  = servoAngles[0];
        servoAngles[0] = servoAngles[0] - deltaAngle1;
        incrementAngle(SERVO1_PIN, oldAngle, servoAngles[0]);
    }
    else if(action == 2){
        oldAngle  = servoAngles[1];
        servoAngles[1] = servoAngles[1] + deltaAngle2;
        incrementAngle(SERVO2_PIN, oldAngle, servoAngles[1]);
    }
    else if(action == 3){
        oldAngle  = servoAngles[1];
        servoAngles[1] = servoAngles[1] - deltaAngle2;
        incrementAngle(SERVO2_PIN, oldAngle, servoAngles[1]);
    }
    else if(action == 4){
        oldAngle  = servoAngles[2];
        servoAngles[2] = servoAngles[2] + deltaAngle3;
        incrementAngle(SERVO3_PIN, oldAngle, servoAngles[2]);
    }
    else if(action == 5){
        oldAngle  = servoAngles[2];
        servoAngles[2] = servoAngles[2] - deltaAngle3;
        incrementAngle(SERVO3_PIN, oldAngle, servoAngles[2]);
    }
}

float s2Dist = 0.0, s3Dist = 0.0;
float prevS2Dist = s2Dist, prevS3Dist = s3Dist;

float getDistance(){
    // reward function
    // rewards forward movement (when sonar1 reports more values than before)
    // penlizes keeping obstacle in front (when sonar2 and sonar3 reports small values which is not zero,
    // as zero might mean there is no object in the range)
    // shift helps us understand if we are moving in the right direction
    // towards avoiding the obstacle
    currentDistance = float(sonar1.ping()); 
    deltaDistance = currentDistance - previousDistance;
    // Serial.print("Prev Distance  ");Serial.println(previousDistance);
    // Serial.print("Current Distance  ");Serial.println(currentDistance);
    if(abs(deltaDistance) < 57.0 || abs(deltaDistance) > 230.0) deltaDistance = 0.0;
    previousDistance = currentDistance;

    float res;
    if(isCollision == 2){
      curD2 = float(sonar2.ping());
      curD3 = float(sonar3.ping());
      float delCur = curD3 - curD2;
      float delPrev = prevD3 - prevD2;
      res = delCur - delPrev;
    }
    else if(isCollision == 1){
      curD2 = float(sonar2.ping());
      curD3 = float(sonar3.ping());
      float delCur = curD2 - curD3;
      float delPrev = prevD2 - prevD3;
      res = delCur - delPrev;
    }
    else{
        res = deltaDistance;
    }
    return res;
}

float getLookahead(){
    float bestValue = -1000000.0; 
    int possible[6] = {0, 0, 0, 0, 0, 0}; 
    if(state1 + 1 != nServoStates1 ){
        bestValue = max(Q[sPrime][0], bestValue);
        possible[0] = 1;
    }
    if(state1 != 0 && isCollision){
        bestValue = max(bestValue, Q[sPrime][1]);
        possible[1] = 1;
    }
    if(state2 + 1 != nServoStates2){
        bestValue = max(bestValue, Q[sPrime][2]);
        possible[2] = 1;
    }
    if(state2 != 0){
        bestValue = max(bestValue, Q[sPrime][3]);
        possible[3] = 1;
    }
    if(state3 + 1 != nServoStates3){
        bestValue = max(bestValue, Q[sPrime][4]);
        possible[4] = 1;
    }
    if(state3 != 0){
        possible[5] = 1;
        bestValue = max(bestValue, Q[sPrime][5]);
    }

    return bestValue;
}
float getLookahead2(){
    float bestValue = -1000000.0; 
    int possible[6] = {0, 0, 0, 0, 0, 0}; 
    if(state1 + 1 != nServoStates1 ){
        bestValue = max(C[sPrime][0], bestValue);
        possible[0] = 1;
    }
    if(state1 != 0 && isCollision){
        bestValue = max(bestValue, C[sPrime][1]);
        possible[1] = 1;
    }
    if(state2 + 1 != nServoStates2){
        bestValue = max(bestValue, C[sPrime][2]);
        possible[2] = 1;
    }
    if(state2 != 0){
        bestValue = max(bestValue, C[sPrime][3]);
        possible[3] = 1;
    }
    if(state3 + 1 != nServoStates3){
        bestValue = max(bestValue, C[sPrime][4]);
        possible[4] = 1;
    }
    if(state3 != 0){
        possible[5] = 1;
        bestValue = max(bestValue, C[sPrime][5]);
    }

    return bestValue;
}
float getLookahead3(){
    float bestValue = -1000000.0; 
    int possible[6] = {0, 0, 0, 0, 0, 0}; 
    if(state1 + 1 != nServoStates1 ){
        bestValue = max(C_[sPrime][0], bestValue);
        possible[0] = 1;
    }
    if(state1 != 0 && isCollision){
        bestValue = max(bestValue, C_[sPrime][1]);
        possible[1] = 1;
    }
    if(state2 + 1 != nServoStates2){
        bestValue = max(bestValue, C_[sPrime][2]);
        possible[2] = 1;
    }
    if(state2 != 0){
        bestValue = max(bestValue, C_[sPrime][3]);
        possible[3] = 1;
    }
    if(state3 + 1 != nServoStates3){
        bestValue = max(bestValue, C_[sPrime][4]);
        possible[4] = 1;
    }
    if(state3 != 0){
        possible[5] = 1;
        bestValue = max(bestValue, C_[sPrime][5]);
    }

    return bestValue;
}


void initializeQC(){
    for(int i = 0; i < nStates; i++)
        for(int j = 0; j < nActions; j++){
            Q[i][j] = 10.0;
            C[i][j] = 10.0;
            C_[i][j] = 10.0;
        }
      

    // C[0][1][1][3] = 1200.0;
    C[getState(0, 1, 1)][3] = 1200.0;
    // C[0][0][1][4] = 1200.0;
    C[getState(0, 0, 1)][4] = 1200.0;
    // C[0][0][2][0] = 1200.0;
    C[getState(0, 0, 2)][0] = 1200.0;
    // C[1][0][2][2] = 1200.0;
    C[getState(1, 0, 2)][2] = 1200.0;
    // C[1][1][2][1] = 1200.0;
    C[getState(1, 1, 2)][1] = 1200.0;
    // C[0][1][2][5] = 1200.0;
    C[getState(0, 1, 2)][5] = 1200.0;
    
    C_[getState(0, 1, 1)][0] = 1200.0;
    C_[getState(1, 1, 1)][3] = 1200.0;
    C_[getState(1, 0, 1)][4] = 1200.0;
    C_[getState(1, 0, 2)][1] = 1200.0;
    C_[getState(0, 0, 2)][2] = 1200.0;
    C_[getState(0, 1, 2)][5] = 1200.0;

}

void writeQToSD(){
  Serial.println(tt);
  delay(500);
  for(int i = 0; i < nStates; i++)
        for(int j = 0; j < nActions; j++){
            Serial.println(Q[i][j]);
            delay(250);
            Serial.println(C[i][j]);
            delay(250);
            Serial.println(C_[i][j]);
            delay(250);
  }
  
}
void writeToQ(){
  // Serial.println("Good");
  while(Serial.available()==0) ;
  tt = Serial.parseFloat(SKIP_ALL, '\n');
  for(int i = 0; i < nStates; i++)
        for(int j = 0; j < nActions; j++){
          while(Serial.available()==0) ;
          Q[i][j] = Serial.parseFloat(SKIP_ALL, '\n');
          while(Serial.available()==0) ;
          C[i][j] = Serial.parseFloat(SKIP_ALL, '\n');
          while(Serial.available()==0) ;
          C_[i][j] = Serial.parseFloat(SKIP_ALL, '\n');

  }
}
void checkForReceive(){
  if(Serial.available()){
    String command = Serial.readString();
    if(command == "receive"){
      writeToQ();
    }
    if(command == "send"){
      delay(3000);
      writeQToSD();
    }
    delay(200);

  }
}
void loop(){
    tt++;
    epsilon = exp(-tt/1000.0);
    checkForReceive();

    if(isCollision == 1){
      action = getAction2();
      goToNextState(action);
      executeAction(action);
      reward = getDistance();
      lookahead = getLookahead2();
      sample = reward + gamma * lookahead;
      C[s][action] += alpha*(sample - C[s][action]);
    }
    else if(isCollision == 2){
      action = getAction3();
      goToNextState(action);
      executeAction(action);
      reward = getDistance();
      lookahead = getLookahead3();
      sample = reward + gamma * lookahead;
      C_[s][action] += alpha*(sample - C_[s][action]);
    }
    else{
      action = getAction();
      goToNextState(action);
      executeAction(action);
      reward = getDistance();
      lookahead = getLookahead();
      sample = reward + gamma * lookahead;
      Q[s][action] += alpha*(sample - Q[s][action]);
    }
    

    s = sPrime;
    if(tt==2) {initializeQC();tt=1200;}


    float s2Dist = sonar2.ping_cm(), s3Dist = sonar3.ping_cm();
    if(s2Dist > 3 && s2Dist < 35 || s3Dist > 3 && s3Dist < 35){
      if(s2Dist > s3Dist || s2Dist == 0)
        isCollision = 1;
      else
        isCollision = 2;
    }
    else{
      isCollision = 0;
    }
    // Serial.print("Sonar 1: "); Serial.println(sonar1.ping_cm());
    // Serial.print("Sonar 2: "); Serial.println(sonar2.ping_cm());
    // Serial.print("Sonar 3: "); Serial.println(sonar3.ping_cm());
    delay(200);
}

void incrementAngle(int servoPin, int fro, int to){
  if(fro < to ){
      for(int i = fro; i <= to; i++){
          pwm.setPWM(servoPin, 0, angleToPulse(i));
          delay(25);
      }
  }
  else{
    for(int i = fro; i >= to; i--){
          pwm.setPWM(servoPin, 0, angleToPulse(i));
          delay(25);
    }      
  }  
}