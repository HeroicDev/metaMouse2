/*
 * Library for Maze
 * 
 */

#ifndef Maze_h
#define Maze_h

#include "Arduino.h"
#include <StackArray.h>

#define NORTH 1
#define SOUTH 2
#define EAST 3
#define WEST 4
/* Used to detect error in the mouse heading */
#define ERROR_DIR -10

#define mazeSize 10

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
};

class Maze
{
  public:
    Maze();
    /* Setup Functions */
    void initializeMaze();
    int calculateCenter(int x, int y);
    int calculateWeight(int x, int y, int desiredX, int desitedY);
    void setUpWheels();
    void checkForWalls();
    void createTestMaze();
    /* End Setup Functions */
    void moveToDir(int direction);
    int getNextDir(Block blk);
    int getMinDistance(Block b);
    void printMaze();
    /* Movement Functions */
    void moveForward();
    void moveBackward();
    void moveLeft();
    void moveRight();
    void fullStop();
    /* End Movement Functions */
//    void floodFill();
    Block getNeighbor(Block blk, int dir);
    void printCords();
    void printCurrent();
    void updateGlobalBlock(Block blk);
    void mapMaze();
    //attributes
    bool isAtCenter();
    void floodfill(Block blk);
    void moveCurrentBlock(int x, int y);
    void sim();
    boolean inBounds(Block b);
    Block getNextDirTwo(Block blk);
    void moveMouseToNextBlock(Block b);
};

#endif

