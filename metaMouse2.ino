#include "Maze.h"

Maze mouse;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  mouse.initializeMaze();
  mouse.createTestMaze();
  //this is here to get around some odd formating issues during simulation
  mouse.printMaze();
//  mouse.setUpWheels();
//  delay(3000);
}

void loop() 
{
//  runLiveTest();
  runSimulation();
  delay(500);
}

void runSimulation()
{
  if (!mouse.isAtCenter()) {
    mouse.printMaze();
    mouse.sim();
  }
  else {
    mouse.printMaze();
    Serial.println("MOUSE IS AT THE CENTER OF THE MAZE. IT MADE IT");
    delay(100000);
  }
}

void runLiveTest()
{
  if (!mouse.isAtCenter()) {
    mouse.checkForWalls();
  } else {
    delay(1000000);
  }
}

