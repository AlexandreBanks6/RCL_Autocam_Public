#include <SevSeg.h>

/*
Title: Ring Collision Detection
Description: Counts the number of collisions between a ring and a wire. The ring is moved over the wire with the da Vinci Research Kit.
Date: November 1st, 2024
Author: Alexandre Banks

States (buttons):
1. "Start" => restarts the display, resets data collection vals, grabs .csv name, writes header & space to csv, starts recording data
2. "Stop" =>  pauses display, writes totals to second line in csv (task duration, # of collisions, and collision time)
3. "Toggle" => toggles the display
.csv format:
row1: Sample #, Task Duration, Collision (binary), Collision Time (cumulative)  
row 2: totals: , total duration, total collisions, total collision time
row 3- end: data

*/
// Includes:
#include "SevSeg.h"



// ############Setup the PINS############

// 7-seg pins
int D1 = 2;
int D2 = 3;
int D3 = 4;
int D4 = 5;

int pinA = 6;
int pinB = 7;
int pinC = 8;
int pinD = 9;
int pinE = 10;
int pinF = 11;
int pinG = 12;
int dec=13;

//Sense pin for collisions
int sensePin=A0; //Pin to measure collisions
int togglePin=A1; //Toggles Display

//7 seg vars
SevSeg sevseg;
byte numDigits=4;
byte digitPins[] = {D1,D2,D3,D4};
byte segmentPins[] = {pinA,pinB,pinC,pinD,pinE,pinF,pinG,dec};
bool resistorsOnSegments = 0; 



//######################CONSTANTS AND GLOBAL VARS#################
int FILTER_THRESHOLD=8; //Threshold for filtering when wire is touched
int DOWN_FILTER_THRESHOLD=5; //Threshold for filtering when wire is released

int collisionCount=0;
bool prevSensorVal=1; //1 in our case means it is not touching ring
bool sensorValue=1;

int filterCount=0;
int downFilter=0;


//Collision duration times
unsigned long collisionTime=0; //Duration of a collision
unsigned long prevTime=0;
unsigned long currTime=0;

//Task duration times
unsigned long taskDuration=0; //Duration of task
unsigned long prevDurationTime=0;
unsigned long currDurationTime=0;

//Booleans for toggle button
int toggleFilter=0;
int toggleState=0; //show on display: 0=duration of task; 1=# of collisions; 2=duration of collisions
bool prevTogglePin=1;
bool senseTogglePin=1;

//Boolean for if start/restart or stopped
bool runningState=0; //Not running at start


int dispTime=0;

//.csv filename string
String csvFilename;
int sampleCount=0;
int curr_collision=0;

String csv_data;

void setup() {

  //####################Setup##################
  sevseg.begin(COMMON_CATHODE,numDigits,digitPins,segmentPins,resistorsOnSegments);
  sevseg.setBrightness(50);
  pinMode(sensePin,INPUT_PULLUP); //Sets the sense pin as input, with an internal pull-up resistor
  pinMode(togglePin,INPUT_PULLUP); //Sets the toggle pin for the display as an active low


  Serial.begin(57600); // open the serial port at 57600 bps, for getting the .csv file name
  Serial.print("Started Serial Monitor with 57600 baud \n");
  //##################Counters#################  
  //Collision Count Params
  collisionCount=0; //Updates count value
  prevSensorVal=1; //Not touching wire (pulled high)
  sensorValue=1; //current sensor value (not touching)
  filterCount=0; //Counter for filter
  downFilter=0; //Filter for releasing the wire

  //Duration of the collision
  collisionTime=0;
  prevTime=millis();
  currTime=millis();

  //Toggle Display Button (still showing on the seven seg)
  toggleFilter=0;
  toggleState=0; //show on display: 0=duration of task; 1=# of collisions; 2=duration of collisions
  prevTogglePin=1;
  senseTogglePin=1;
  dispTime=0;

  //Are we running or not
  runningState=0; 


  //Task duration
  prevDurationTime=millis();
  currDurationTime=millis();
  taskDuration=0;


  //Sets the seven seg to zero
  sevseg.setNumber(taskDuration,-1);
  sevseg.refreshDisplay();

  sampleCount=0;
  curr_collision=0; //Binary value where 0 is coliding, and 1 is not

}

void loop() {


  if(!runningState) //Haven't started the task
  {
    Serial.read(); //Clears the input buffer
    //Grab the .csv name we want to write to:
    Serial.print("************************\n");
    Serial.print("Enter CSV name to save data to (include .csv): \n");
    while (Serial.available()==0){ //Waits until user inputs .csv name
    }
    //Grabs the file name
    csvFilename=Serial.readString();
    Serial.print("CSV Filename: \n");
    Serial.print(csvFilename);
    Serial.print("\n");

    Serial.print("Press Enter to Start Task: \n");

    while (Serial.available()==0){ //Waits until user starts task via pressing enter
    }
    Serial.read(); //Clears the input buffer
    

    Serial.print("Task Started\n");
    Serial.print("Press Enter to Stop Task: \n");

    //Reseting Booleans and counters

    //Reseting booleans
    collisionCount=0; //Starts with 0 collisions
    prevSensorVal=1; //Sets the previous and current sensor values to 1
    sensorValue=1;
    filterCount=0; //filter for touching wire
    downFilter=0; //Filter for releasing the wire

    //Duration of the collision
    collisionTime=0;
    prevTime=millis();
    currTime=millis();


    //Task duration
    prevDurationTime=millis();
    currDurationTime=millis();
    taskDuration=0;

    sevseg.setNumber(taskDuration,1);
    sevseg.refreshDisplay();

    runningState=1;
    sampleCount=0;
    curr_collision=0;

  }


  if(runningState) //Currently running the task
  {
    curr_collision=0;

    //*********Toggle Button************
    senseTogglePin=digitalRead(togglePin);
    if((senseTogglePin==0)&&(prevTogglePin==1)){ //Pressing toggle button
      toggleFilter++;
    }
    else{
      toggleFilter=0;
      prevTogglePin=senseTogglePin;
    }
    if(toggleFilter>FILTER_THRESHOLD){ //Update toggle state
      toggleState++;
      if(toggleState==3){
        toggleState=0;
      }
      toggleFilter=0;
      prevTogglePin=senseTogglePin;
    }



    //###########Detecting Collisions################
    currDurationTime=millis();
    taskDuration=taskDuration+(currDurationTime-prevDurationTime);
    prevDurationTime=currDurationTime;
    //Read the sense pin from the collision
    sensorValue=digitalRead(sensePin); //Gets the value in binary

    //###########Determining State of Collision################
    if((sensorValue==0)&&(prevSensorVal==1)){ //Wire touched, increment filter counter
      filterCount++;

    }
    else if(filterCount<=FILTER_THRESHOLD){
      prevSensorVal=sensorValue;
      filterCount=0;
    }

    if(filterCount>FILTER_THRESHOLD){ // Currently touching the wire
      curr_collision=1;
      if((sensorValue==0)&&(prevSensorVal==1)){ //Increments collision counter (this is start of collision)
        collisionCount++;
        prevTime=millis(); //Gets time at the start of the collision
      }

      if((sensorValue==1)&&(prevSensorVal==0)){ //Wire is off of ring
        downFilter++;
      }
      else{
        prevSensorVal=sensorValue; //updates sensor value
        downFilter=0;
      }
      
      if(downFilter>DOWN_FILTER_THRESHOLD){ // We released the ring
      filterCount=0;
      downFilter=0;
      prevSensorVal=sensorValue;
      }
      else{ //If ring is not released, increment collision time since start of task
        currTime=millis();
        collisionTime=collisionTime+(currTime-prevTime); //Time of collision since the start of the task
        prevTime=currTime;

      }

      



    }

    //Sending Data Out over Serial
    
    csv_data=String(sampleCount)+","+String(taskDuration)+","+String(curr_collision)+","+String(collisionCount)+","+String(collisionTime)+"\n";    
    Serial.print(csv_data);
    sampleCount++;


    //Checking if enter was pressed
    if(Serial.available()) //Enter was pressed
    {
      runningState=0;
      Serial.print("Task Stopped\n");

    }
    

  }

  //Display on seven seg
  switch(toggleState){
    case 0: //Duration of task

      dispTime=taskDuration/100; //Show one spot after decimal
      dispTime=round(dispTime);
      sevseg.setNumber(dispTime,1);

      break;
    case 1: //Number of collisions
      sevseg.setNumber(collisionCount,-1);
      break;
    case 2: //Duration of collisions
      //Converting to show two spots after decimanl
      dispTime=collisionTime/10; 
      dispTime=round(dispTime);
      sevseg.setNumber(dispTime,2);
      break;
    default:
      dispTime=taskDuration/100;
      dispTime=round(dispTime);
      sevseg.setNumber(dispTime,1);
  }

  sevseg.refreshDisplay();
  delay(2);

}
