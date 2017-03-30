/*
 * Library for Maze
 * 
 */

#ifndef Maze_h
#define Maze_h

#include "Arduino.h"
#include <QueueList.h>
#include <StackArray.h>

#define NORTH 1
#define SOUTH 2
#define EAST 3
#define WEST 4
#define ERROR_DIR -10

#define mazeSize 6

struct Block
{
  int x = 0;
  int y = 0;
  int weight = -1;
  boolean northWall = false;
  boolean southWall = false;
  boolean eastWall = false;
  boolean westWall = false;
  int dir = -1;

  boolean inBounds() {
    boolean in = ( (x > 0 && x < mazeSize) && (y > 0 && y < mazeSize) );
    return in;
  }
};

class Maze
{
  public:
    Maze();
    void setUpWheels();
    void checkForWalls();
    void move();
    void fullStop();
    void makeMove();
    int findNeightborWithLowestWeight(bool returnMin);
    void moveToDir(int direction);
    int getNextDir(Block blk);
    void createTestMaze();
    void initializeMaze();
    int calculateCenter(int x, int y);
    int calculateWeight(int x, int y, int desiredX, int desitedY);
    int getMinDistance(Block b);
    void pushNeighborsToStack(Block b);
    void printMaze();
    void printStack();
    void moveForward();
    void moveBackward();
    void moveLeft();
    void moveRight();
    void floodFill();
    Block getNeighbor(Block blk, int dir);
    void printCords();
    void printCurrent();
    void printPop(Block b);
    void updateGlobalBlock(Block blk);
    void floodUpdate(Block b, int md);
    void printPush(Block nB);
    Block getOpenNeighbor(Block blk, int lastDir); // Modified THIS
    void mapMaze();
    void modFlood();
    //attributes
    bool foundCenter();
    void modifiedFill(Block blk);
    void moveCurrentBlock(int x, int y);
    void sim();
    boolean inBounds(Block b);
    Block getNextDirTwo(Block blk);
    void moveMouseToNextBlock(Block b);
};

#endif

