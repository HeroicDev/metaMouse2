#include "Maze.h"
#include <QueueList.h>
#include <StackArray.h>

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
  delay(1000); //1 seconds
}

void runTest()
{
  mouse.makeMove();
  delay(400);
}

void mapMaze()
{
  //while we have not found the center, keep mapping
  while (!mouse.foundCenter()) {
    mouse.checkForWalls();
    mouse.move();
    delay(2000);
  }
}

void simulate()
{
  mouse.printMaze();
  mouse.sim();
}

