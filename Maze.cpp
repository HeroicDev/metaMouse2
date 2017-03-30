#include "Maze.h"

StackArray <Block> queue;

/* Parameters for sensors and wheels */
int WallDist = 380;
int NoWall = 260;
int target = 330; //3cm
//Pin Assignment
byte PWMA = 3;   //PWM control for motor outputs 1 and 2 is on digital pin 3
byte PWMB = 11;  //PWM control for motor outputs 3 and 4 is on digital pin 11
byte DIRA = 13;  //direction control for motor outputs 1 and 2 is on digital pin 12
byte DIRB = 12;  //direction control for motor outputs 3 and 4 is on digital pin 13
byte AW0 = 0;     // Front Infrared Sensor (Right lead) connected to analog pin 0
byte AW1 = 1;     // Left Infrared Sensor (Right lead) connected to analog pin 1
byte AW2 = 2;     // Right Infrared Sensor (Right lead) connected to analog pin 2
byte AW3 = 3;    //Back infared sensor connected to analog pin 3
byte frontS, rightS, leftS, backS = 0;    // variable to store the value read
byte speed = 60; //127;
/* End parameters for sensors and wheels */

/* Maze parameters */
Block currentBlock;
Block maze[mazeSize][mazeSize]; 
int currentX; //x location
int currentY; //y location
int nextDir = NORTH;
int currentDir; //current direction
int minDist;
/* End maze parameters */

StackArray <Block> modQ;

Maze::Maze()
{
  //empty constructor
}

bool Maze::isAtCenter()
{
  if (currentBlock.weight == 0) {
    return true;
  }

  return false;
}

void Maze::setUpWheels()
{
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(DIRA, OUTPUT);
  pinMode(DIRB, OUTPUT);
  pinMode(AW0, INPUT);   //Probably Not Be Nesassery!
  pinMode(AW1, INPUT);   
  pinMode(AW2, INPUT);
  pinMode(AW3, INPUT);
//  delay(3000);
  Serial.print("Done setting up wheels and sensors");
}

void Maze::checkForWalls()
{
  frontS = analogRead(AW0);
  leftS = analogRead(AW1);
  rightS = analogRead(AW2);
  backS = analogRead(AW3);

  if (rightS < target) {
    //front wall
    currentBlock.northWall = true;
    
  }

  if (leftS < target) {
    //west wall
    currentBlock.westWall = true;
  }

  if (rightS < target) {
    //east wall
    currentBlock.eastWall = true;
  }

  if (backS < target) {
    //south wall
    currentBlock.southWall = true;
  }

  sim();
}

/* Simulation test function Makes a move*/
void Maze::sim()
{
  modifiedFill(currentBlock);
//  nextDir = getNextDir(currentBlock);
  Block nextB = getNextDirTwo(currentBlock);
  moveMouseToNextBlock(nextB);
  //moveToDir(nextDir);
  //check current block
   
}

void Maze::moveMouseToNextBlock(Block b)
{
  int dir = -10; //error
  if (b.y < currentBlock.y && b.x == currentBlock.x) {
    dir = NORTH;
  }
  else if (b.y > currentBlock.y && b.x == currentBlock.x) {
    dir = SOUTH;
  } 
  else if (b.x > currentBlock.x && b.y == currentBlock.y) {
    dir = EAST;
  }
  else if (b.x < currentBlock.x && b.y == currentBlock.y){
    dir = WEST;
  }
  
  moveToDir(dir);
}

void Maze::moveToDir(int direction)
{
  switch (direction) {
    case NORTH:
      moveForward();
      break;
    case SOUTH:
      moveBackward();
      break;
    case EAST:
      moveRight();
      break;
    case WEST:
      moveLeft();
      break;
    case ERROR_DIR:
      //go north
      Serial.println("THERE IS AN ERROR IN THE NEXT DIRECTION. GOING NORTH");
      moveForward();
      break;
  }
}

/*
 * Returns the next direction the mouse should head to.
 * Tries to go from larger weights to smaller weights
 */
int Maze::getNextDir(Block blk)
{
  if (!blk.northWall) {
    Block nb = getNeighbor(blk, NORTH);
    if (inBounds(nb) && blk.weight > nb.weight) {
      return NORTH;
    }
  }

  if (!blk.southWall) {
    Block nb = getNeighbor(blk, SOUTH);
    if (inBounds(nb) && blk.weight > nb.weight) {
      return SOUTH;
    }
  }
  
  if (!blk.eastWall) {
    Block nb = getNeighbor(blk, EAST);
    if (inBounds(nb) && blk.weight > nb.weight) {
      return EAST;
    }
  }
  
  if (!blk.westWall) {
    Block nb = getNeighbor(blk, WEST);
    if (inBounds(nb) && blk.weight > nb.weight) {
      return WEST;
    }
  }
}

Block Maze::getNextDirTwo(Block b)
{
  Block minN;
  int md = mazeSize * mazeSize;

  if (!b.northWall) {
    Block n = getNeighbor(b, NORTH);
    if (inBounds(n) && n.weight < md) {
      md = n.weight;
      minN = n;
    }
  }

  if (!b.southWall) {
    Block n = getNeighbor(b, SOUTH);
    if (inBounds(n) && n.weight < md) {
      md = n.weight;
      minN = n;
    }
  }

  if (!b.westWall) {
    Block n = getNeighbor(b, WEST);
    if (inBounds(n) && n.weight < md) {
      md = n.weight;
      minN = n;
    }
  }
  
  if (!b.eastWall) {
    Block n = getNeighbor(b, EAST);
    if (inBounds(n) && n.weight < md) {
      md = n.weight;
      minN = n;
    }
  }
  
  return minN;
}

void Maze::createTestMaze()
{
  /* Row 0 */
  maze[0][2].northWall = true;
  maze[0][2].southWall = true;
  maze[0][3].northWall = true;
  maze[0][3].southWall = true;
  maze[0][4].northWall = true;
  maze[0][4].southWall = true;

  /* Row 1 */
  maze[1][0].westWall = true;
  maze[1][0].eastWall = true;
  maze[1][1].westWall = true;
  maze[1][2].northWall = true;
  maze[1][2].southWall = true;
//  maze[1][2].eastWall = true;
  maze[1][3].northWall = true;
  maze[1][3].southWall = true;
  maze[1][4].northWall = true;
  maze[1][5].southWall = true;

  /* Row 2 */
  maze[2][0].westWall = true;
  maze[2][0].eastWall = true;
  maze[2][1].westWall = true;
  maze[2][1].eastWall = true;
  maze[2][1].southWall = true;
  maze[2][2].northWall = true;
  maze[2][2].westWall = true;
  maze[2][3].northWall = true;
  maze[2][5].northWall = true;
  maze[2][5].southWall = true;

  /* Row 3 */
  maze[3][0].westWall = true;
  maze[3][0].southWall = true;
  maze[3][1].northWall = true;
  maze[3][1].eastWall = true;
  maze[3][2].westWall = true;
  maze[3][2].southWall = true;
  maze[3][3].southWall = true;
  maze[3][3].eastWall = true;
  maze[3][4].westWall = true;
  maze[3][4].southWall = true;
  maze[3][5].northWall = true;

  /* Row 4 */
  maze[4][0].westWall = true;
  maze[4][0].northWall = true;
  maze[4][1].eastWall = true;
  maze[4][2].westWall = true;
  maze[4][2].northWall = true;
  maze[4][3].northWall = true;
  maze[4][3].eastWall = true;
  maze[4][4].westWall = true;
  maze[4][4].northWall = true;

  /* Row 5 */
  maze[5][0].westWall = true;
  maze[5][0].southWall = true;
  maze[5][0].eastWall = true;
  maze[5][1].westWall = true;
  maze[5][1].southWall = true;
  maze[5][2].eastWall = true;
  maze[5][2].southWall = true;
  maze[5][3].westWall = true;
  maze[5][3].southWall = true;
  maze[5][4].eastWall = true;
  maze[5][4].southWall = true;
  maze[5][5].westWall = true;
  maze[5][5].southWall = true;
}

/*
* Instansiates the maze.
* It sets the weight and the cordinate of each block.
*/
void Maze::initializeMaze()
{
  for (int y = 0; y < mazeSize; y++) {
    for (int x = 0; x < mazeSize; x++) {
      //left walls
      if (x == 0 && y < mazeSize) {
        maze[y][x].westWall = true;
      }
      //right walls
      if (x == (mazeSize - 1) && y < mazeSize) {
        maze[y][x].eastWall = true;
      }
      //top wall
      if (y == 0 && x < mazeSize) {
        maze[y][x].northWall = true;
      }
      //bottom wall
      if (y == (mazeSize - 1) && x < mazeSize) {
        maze[y][x].southWall = true;
      }

      maze[y][x].y = y;
      maze[y][x].x = x;
      maze[y][x].weight = calculateCenter(x, y);
    }
  }
  
  /* Set current position and currentBlock */
  currentX = 0;
  currentY = 5;
  currentDir = NORTH;

  moveCurrentBlock(currentX, currentY);
}

//finds how far a block is from the center
//this distance is the weight
int Maze::calculateCenter(int x, int y)
{
  int center = (mazeSize / 2);
  int weight = 0;
  int top = center - 1;

  //top side of maze
  if (y <= top) {
    //top left maze
    if (x <= top) {
      weight = calculateWeight(x, y, (center - 1), (center - 1));
    }
    else {
      //top right
      weight = calculateWeight(x, y, center, (center - 1));
    }
  }
  //bottom of maze
  else {
    //bottom left
    if (x <= top) {
      weight = calculateWeight(x, y, (center - 1), center);
    }
    else {
      //bottom right
      weight = calculateWeight(x, y, center, center);
    }
  }
  return weight;
}

/*
* Calcuate the distance of a cell from the center
*/
int Maze::calculateWeight(int x, int y, int desiredX, int desitedY)
{
  return abs(desitedY - y) + abs(desiredX - x);
}

/**
 * Returns the distance of the neighbors
 */
int Maze::getMinDistance(Block b)
{
  int md = mazeSize * mazeSize; //sizeof(maze) * sizeof(maze);

  if (!b.northWall) {
    Block n = getNeighbor(b, NORTH);
    if (inBounds(n) && n.weight < md) {
      md = n.weight;
    }
  }

  if (!b.southWall) {
    Block n = getNeighbor(b, SOUTH);
    if (inBounds(n) && n.weight < md) {
      md = n.weight;
    }
  }

  if (!b.westWall) {
    Block n = getNeighbor(b, WEST);
    if (inBounds(n) && n.weight < md) {
      md = n.weight;
    }
  }
  
  if (!b.eastWall) {
    Block n = getNeighbor(b, EAST);
    if (inBounds(n) && n.weight < md) {
      md = n.weight;
    }
  }
  
  return md;
}

/*
* Prints out the maze for debugging purposes
*/
void Maze::printMaze()
{
//  Serial.print("\t");
  Serial.println("\n\t");
  for (int y = 0; y < mazeSize; y++) {
    //first loop print north wall's
//    Serial.print("\t");
    for (int x = 0; x < mazeSize; x++) {
      Serial.print("+");
      if (maze[y][x].northWall) {
        Serial.print("-------");
      }
      else {
        Serial.print("       ");
      }
    }
    Serial.print("+\n\t");

    //second loop, print left/right wall and weight
    for (int x = 0; x < mazeSize; x++) {
      //left wall
      if (x == 0 && maze[y][x].westWall) {
        Serial.print("|");
      }

      int w = maze[y][x].weight;
      
      Serial.print("  ");
      if (w < 10) {
        Serial.print(" ");
      }
      
      Serial.print(w);
      Serial.print("  ");

      if (currentX == x && currentY == y) {
        Serial.print("^");
      }
      else {
        Serial.print(" ");
      }

      if (maze[y][x].eastWall) {
        Serial.print("|");
      }
      else {
        Serial.print(" ");
      }
    }
    Serial.print("\n\t");
  }

  int end = 0;
  while (end != mazeSize) {
    Serial.print("+-------");
    end++;
  }
  Serial.print("+\n\t");
}

void Maze::printCords()
{
    Serial.println("(x, y)");
  
    for (int y = 0; y < mazeSize; y++) {
    //first loop print north wall's
    for (int x = 0; x < mazeSize; x++) {
      Serial.print("+");
      if (maze[y][x].northWall) {
        Serial.print("----------");
      }
      else {
        Serial.print("          ");
      }
    }
    Serial.print("+\n");

    //second loop, print left/right wall and weight
    for (int x = 0; x < mazeSize; x++) {
      //left wall
      if (x == 0 && maze[y][x].westWall) {
        Serial.print("|");
      }

      Serial.print("  (");
      Serial.print(maze[y][x].x);
      Serial.print(", ");
      Serial.print(maze[y][x].y);
      Serial.print(")  ");

      if (maze[y][x].eastWall) {
        Serial.print("|");
      }
      else {
        Serial.print(" ");
      }
    }
    Serial.print("\n");
  }

  int end = 0;
  while (end != mazeSize) {
    Serial.print("+----------");
    end++;
  }
  Serial.print("+\n");
}

//move the mouse forward
//idea: these type of functions should also update the current x or y position instead of the trackPosition function
void Maze::moveForward()
{
  currentY = currentY - 1;
  currentX = currentX;
  moveCurrentBlock(currentX, currentY);
  digitalWrite(DIRA, LOW);
  digitalWrite(DIRB, LOW);
  analogWrite(PWMA, speed);
  analogWrite(PWMB, speed);
  delay(5000);
}

void Maze::moveBackward()
{
  currentY = currentY + 1;
  currentX = currentX;
  moveCurrentBlock(currentX, currentY);
  digitalWrite(DIRA, HIGH);
  digitalWrite(DIRB, HIGH);
  analogWrite(PWMA, speed);
  analogWrite(PWMB, speed);
  delay(5000);
}

void Maze::moveLeft()
{
  //update currentX and currentY;
  currentX = currentX - 1;
  currentY = currentY;
  moveCurrentBlock(currentX, currentY);
  digitalWrite(DIRA, HIGH);
  digitalWrite(DIRB, LOW);
  analogWrite(PWMA, speed);
  analogWrite(PWMB, speed);
  delay(5000);
}

void Maze::moveRight()
{
  currentX = currentX + 1;
  currentY = currentY;
  moveCurrentBlock(currentX, currentY);
  digitalWrite(DIRA, LOW);
  digitalWrite(DIRB, HIGH);
  analogWrite(PWMA, speed);
  analogWrite(PWMB, speed);
  delay(5000);
}

void Maze::fullStop()
{
  int s = 20;
  digitalWrite(DIRB, HIGH);
  digitalWrite(DIRA, HIGH);
  analogWrite(PWMA, s);
  analogWrite(PWMB, s);  
}

/*
 * Set's the global current block to the x and y passed in
*/
void Maze::moveCurrentBlock(int x, int y)
{
  currentBlock = maze[y][x];
}

//Get the neighbor based on desired direction
Block Maze::getNeighbor(Block blk, int dir)
{
  int x, y;
  switch(dir) {
    case NORTH:
      x = blk.x;
      y = blk.y - 1;
      break;
    case WEST:
      x = blk.x - 1;
      y = blk.y;
      break;
    case EAST:
      x = blk.x + 1;
      y = blk.y;
      break;
    case SOUTH:
      x = blk.x;
      y = blk.y + 1;
      break;
  }

  Block nb = maze[y][x];
  nb.dir = dir;
  return nb;
}

void Maze::updateGlobalBlock(Block blk)
{
  maze[blk.y][blk.x].weight = blk.weight;
}

/*
void Maze::modFlood()
{
  //get min distance
  while (!queue.isEmpty()) {
    Block blk = queue.pop();
    Block nb;
    int minD = findNeightborWithLowestWeight(true);
    //end getting min distance

    Serial.print("min distance: ");
    Serial.println(minD);

    if (blk.weight != 0 && blk.weight != (minD + 1)) {
      blk.weight = minD + 1;
      updateGlobalBlock(blk);
      pushNeighborsToStack(blk);
    }
  }
} */

/*
 * Block blk - Current Block
 */
void Maze::modifiedFill(Block blk)
{
  modQ.push(blk);

  while (!modQ.isEmpty()) {
    Block b = modQ.pop();
    int minD = getMinDistance(b);

    if (b.weight - 1 != minD) {
      b.weight = (minD + 1);
      updateGlobalBlock(b);

      //push open neighbors to stack
      if (!b.westWall) {
        Block nB = getNeighbor(b, WEST);
        if (nB.weight != 0 && inBounds(nB)) {
          modQ.push(nB);
        }
      }
    
      if (!b.northWall) {
        Block nB = getNeighbor(b, NORTH);
        if (nB.weight != 0 && inBounds(nB)) {
          modQ.push(nB);
        }
      }
    
      if (!b.eastWall) {
        Block nB = getNeighbor(b, EAST);
        if (nB.weight != 0 && inBounds(nB)) {
          modQ.push(nB);
        }
      }
    
      if (!b.southWall) {
        Block nB = getNeighbor(b, SOUTH);
        if (nB.weight != 0 && inBounds(nB)) {
          modQ.push(nB);
        }
      }
    }
  }
}

/*
 * Checks to see if the block is within the maze boundaries
 */
boolean Maze::inBounds(Block b)
{
  if ( (b.x >= mazeSize) || (b.y >= mazeSize) || (b.x < 0) || (b.y < 0) ) {
    return false; 
  }

  return true;
}

