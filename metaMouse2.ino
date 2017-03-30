#include "Maze.h"
#include <QueueList.h>

Maze mouse;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  mouse.initializeMaze();
//  mouse.createTestMaze();
  mouse.setUpWheels();
  delay(3000);
}

void loop() 
{
  if (!mouse.isAtCenter()) {
    testRun();
  } else {
    delay(1000000);
  }
  
  

   /*
  if (!mouse.isAtCenter()) {
    simulate(); 
  }
  else {
    mouse.printMaze();
    Serial.println("MOUSE IS AT THE CENTER OF THE MAZE. IT MADE IT");
    delay(10000);
  } */
  
  //todo: add checker for the center of the maze so we can stop the mouse
  delay(500); //1 seconds
}

void simulate()
{
//  mouse.printMaze();
  mouse.sim();
}

void testRun()
{
  mouse.checkForWalls();
}

