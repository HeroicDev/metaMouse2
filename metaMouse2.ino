#include "Maze.h"
#include <QueueList.h>

Maze mouse;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  mouse.initializeMaze();
  mouse.createTestMaze();
//  mouse.setUpWheels();
}

void loop() 
{
//  runTest(); //todo: remove this
  simulate();
  //todo: add checker for the center of the maze so we can stop the mouse
  delay(500); //1 seconds
}

void simulate()
{
  mouse.printMaze();
  mouse.sim();
}

