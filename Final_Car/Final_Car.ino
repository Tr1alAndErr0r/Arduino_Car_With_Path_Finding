#include <StandardCplusplus.h>
#include <system_configuration.h>
#include <unwind-cxx.h>
#include <utility.h>
#include <vector>
#include <limits.h>
#include <stdio.h>
#include "Adafruit_VL6180X.h"

using namespace std;

// defining laser pins
Adafruit_VL6180X rightLaser;
Adafruit_VL6180X middleLaser;
Adafruit_VL6180X leftLaser;
byte rightLaserSHDNPin = 3; //ShutDown pin
byte middleLaserSHDNPin = 4;
byte leftLaserSHDNPin = 6;

// defining motor pins
int pinI1 = 8; //define I1 interface
int pinI2 = 11; //define I2 interface 
int pinI3 = 12; //define I3 interface 
int pinI4 = 13; //define I4 interface 
int speedpinA = 9; //enable motor A
int speedpinB = 10; //enable motor B

int normSpeed = 120; //define the speed of motor
int turnSpeed = 150;

// defining thresholds
int frontDistance = 30; // threshold for front distance
int crashSideDistance = 30; // threshold for when side sensor hits side
int sideLowBound = 90; // for keeping center line
int sideHighBound = 100;
int infiniteDistance = 255; // threshold for node detection

// defining times for turns
int turnAroundTime = 1100; // time for 360
int ninetyDegreeTurnTime = 450;// was at 345
int shiftTime = 100;  

// to initialize orientation/map list
const int node_count = 14;
char directions[node_count][node_count];
int graph[node_count][node_count];

int no_edge = 50; // if an edge does not exist
vector<int> PATH; // the path to get to destination
int destination = 13; // end node

int atThisNode = 0; // to keep track of which node car is at
int nextNode = 0; // to keep track of which node has to go to
int previousNode = 0;
char currentOrientation = 'N'; // to keep track of which way car is facing
char nextOrientation = 'N';

int rightValue = 0;
int middleValue = 0;
int leftValue = 0;
bool doneFlag = false;
/*
//Testing
int testArray[10];
int testCounter = 1;
*/
void setup() 
{
  // set shutdown pins to output mode
  pinMode(rightLaserSHDNPin, OUTPUT);
  pinMode(middleLaserSHDNPin, OUTPUT);
  pinMode(leftLaserSHDNPin, OUTPUT);
  Wire.begin();
  Serial.begin(9600);

  // shutdown lasers to start
  digitalWrite(rightLaserSHDNPin, LOW);
  digitalWrite(middleLaserSHDNPin, LOW);
  digitalWrite(leftLaserSHDNPin, LOW);

  // Initialize right laser
  digitalWrite(rightLaserSHDNPin, HIGH);
  delay(500);
  rightLaser.begin(0x80);

  // Initialize middle laser
  digitalWrite(middleLaserSHDNPin, HIGH);
  delay(500);
  middleLaser.begin(0x82);

  // Initialize left laser
  digitalWrite(leftLaserSHDNPin, HIGH);
  delay(500);
  leftLaser.begin(0x84);
  
  //motor setup
  pinMode(pinI1,OUTPUT);
  pinMode(pinI2,OUTPUT);
  pinMode(speedpinA,OUTPUT);
  pinMode(pinI3,OUTPUT);
  pinMode(pinI4,OUTPUT);
  pinMode(speedpinB,OUTPUT);

  // prepopulate directions list
  for(int i = 0; i < node_count; i++)
  {
    for(int j = 0; j < node_count; j++)
    {
      directions[i][j] = 'k'; // k means edge does not exist
    }
  } 
  // add relevant directions
  directions[0][1] = 'N';
  directions[0][2] = 'W';
  
  directions[1][0] = 'S';
  directions[1][4] = 'W';
  
  directions[2][0] = 'E';
  directions[2][3] = 'N';
  directions[2][8] = 'W';

  directions[3][2] = 'S';
  directions[3][4] = 'N';
  directions[3][8] = 'W';

  directions[4][1] = 'E';
  directions[4][3] = 'S';
  directions[4][5] = 'N';
  directions[4][9] = 'W';

  directions[5][4] = 'S';
  directions[5][7] = 'W';

  directions[6][7] = 'N';
  directions[6][11] = 'W';

  directions[7][5] = 'E';
  directions[7][6] = 'S';

  directions[8][2] = 'T'; // represents the curve is really more SE
  directions[8][3] = 'E';
  directions[8][9] = 'N';

  directions[9][4] = 'E';
  directions[9][8] = 'S';
  directions[9][10] = 'N';
  
  directions[10][9] = 'S';
  directions[10][11] = 'N';
  directions[10][12] = 'W';
  
  directions[11][6] = 'E';
  directions[11][10] = 'S';

  directions[12][10] = 'E';
  directions[12][13] = 'N';

  directions[13][12] = 'S';

  // prepopulate all edges to 'no_edge'
  for(int i = 0; i < node_count; i++)
  {
    for(int j = 0; j < node_count; j++)
    {
      graph[i][j] = no_edge;
    }
  } 
  // populate edges
  graph[0][1] = 3;
  graph[0][2] = 2;
  
  graph[1][0] = 3;
  graph[1][4] = 2;

  graph[2][0] = 2;
  graph[2][3] = 1;

  graph[3][2] = 1;
  graph[3][4] = 1;
  graph[3][8] = 3;

  graph[4][1] = 1;
  graph[4][3] = 1;
  graph[4][5] = 3;
  graph[4][9] = 3;

  graph[5][4] = 3;
  graph[5][7] = 1;

  graph[6][7] = 1;
  graph[6][11] = 1;

  graph[7][5] = 1;
  graph[7][6] = 1;

  graph[8][3] = 3;
  graph[8][9] = 1;

  graph[9][4] = 3;
  graph[9][8] = 1;
  graph[9][10] = 1;

  graph[10][9] = 1;
  graph[10][11] = 1;
  graph[10][12] = 1;

  graph[11][6] = 1;
  graph[11][10] = 1;

  graph[12][10] = 1;
  graph[12][13] = 4;

  graph[13][12] = 4;

  /*
  // for testing optimal
  testArray[0] = 0;
  testArray[1] = 1;
  testArray[2] = 4;
  testArray[3] = 9;
  testArray[4] = 10;
  testArray[5] = 12;
  testArray[6] = 13;
  */
  /*
  testArray[0] = 4;
  testArray[1] = 5;
  testArray[2] = 7;
  testArray[3] = 6;
  testArray[4] = 11;
  testArray[5] = 10;
  testArray[6] = 12;
  testArray[7] = 13;
  */
}

void forward(int speedL, int speedR) //go forward
{
  analogWrite(speedpinA,speedL); // left wheel speed
  analogWrite(speedpinB,speedR); // right wheel speed
  digitalWrite(pinI4,LOW);//turn DC Motor B move anticlockwise
  digitalWrite(pinI3,HIGH);
  digitalWrite(pinI2,LOW);//turn DC Motor A move anticlockwise
  digitalWrite(pinI1,HIGH);
}
void backward(int speedL, int speedR) //go backward
{
  analogWrite(speedpinA,speedL); // left wheel speed
  analogWrite(speedpinB,speedR); // right wheel speed
  digitalWrite(pinI4,HIGH);//turn DC Motor B move clockwise
  digitalWrite(pinI3,LOW);
  digitalWrite(pinI2,HIGH);//turn DC Motor A move clockwise
  digitalWrite(pinI1,LOW);
}
void left(int speedT) //go left
{
  analogWrite(speedpinA,speedT);//input a simulation value to set the speed
  analogWrite(speedpinB,speedT);
  digitalWrite(pinI4,LOW);//turn DC Motor B move anticlockwise
  digitalWrite(pinI3,HIGH);
  digitalWrite(pinI2,HIGH);//turn DC Motor A move clockwise
  digitalWrite(pinI1,LOW);
}
void right(int speedT) //go right
{
  analogWrite(speedpinA,speedT);//input a simulation value to set the speed
  analogWrite(speedpinB,speedT);
  digitalWrite(pinI4,HIGH);//turn DC Motor B move clockwise
  digitalWrite(pinI3,LOW);
  digitalWrite(pinI2,LOW);//turn DC Motor A move anticlockwise
  digitalWrite(pinI1,HIGH);
}
void stop() //stop moving
{
  digitalWrite(speedpinA,LOW);// Unenble the pin, to stop the motor.
  digitalWrite(speedpinB,LOW);
}

void turnAround(int speedT)
{
  left(speedT);
  delay(turnAroundTime);
  stop();
}
 
void turnRightTimed(int speedT, int timeToTurnT)
{
  right(speedT);
  delay(timeToTurnT);
  stop();
}

void turnLeftTimed(int speedT, int timeToTurnT)
{
  left(speedT);
  delay(timeToTurnT);
  stop();
}


// find which direction to go and turn into that direction
void whichDirectionToGo()
{
  previousNode = atThisNode;
  driver(atThisNode, destination);
  PATH.erase(PATH.begin());
  nextNode = PATH.front();
  nextOrientation = directions[atThisNode][nextNode];

  if(currentOrientation == 'N')
  {
    switch(nextOrientation)
    {
      case 'W': turnLeftTimed(turnSpeed, ninetyDegreeTurnTime);
                break;  
      case 'E': turnRightTimed(turnSpeed, ninetyDegreeTurnTime);
                break;
      case 'S': turnAround(normSpeed);
                break;
      default: 
                break;
    }
  }

  else if(currentOrientation == 'S')
  {
    switch(nextOrientation)
    {
      case 'W': turnRightTimed(turnSpeed, ninetyDegreeTurnTime);
                break;  
      case 'E': turnLeftTimed(turnSpeed, ninetyDegreeTurnTime);
                break;
      case 'N': turnAround(normSpeed);
                break;
      default:
                break;
    }
  }

  else if(currentOrientation == 'E')
  {
    switch(nextOrientation)
    {
      case 'N': turnLeftTimed(turnSpeed, ninetyDegreeTurnTime);
                break;  
      case 'W': turnAround(normSpeed);
                break;
      case 'S': turnRightTimed(turnSpeed, ninetyDegreeTurnTime);
                break;
      default:
                break;
    }
  }

  else if(currentOrientation == 'W')
  {
    switch(nextOrientation)
    {
      case 'N': turnRightTimed(turnSpeed, ninetyDegreeTurnTime);
                break;  
      case 'E': turnAround(normSpeed);
                break;
      case 'S': turnLeftTimed(turnSpeed, ninetyDegreeTurnTime);
                break;
      default:
                break;
    }
  }
  stop();
  delay(200);
  forward(normSpeed, normSpeed);
  delay(850);
  
  //if stuck on wall
  if(rightLaser.readRange() < crashSideDistance || leftLaser.readRange() < crashSideDistance)
  {
    while(rightLaser.readRange() < crashSideDistance || leftLaser.readRange() < crashSideDistance)
    {  
      stop();
      backward(normSpeed, normSpeed);
      delay(400);
      if(rightLaser.readRange() < crashSideDistance)
      {
        stop();
        left(normSpeed);
        delay(100);
        stop();
        forward(normSpeed,normSpeed);
        delay(300);
      }
      if(leftLaser.readRange() < crashSideDistance)
      {
        stop();
        right(normSpeed);
        delay(100);
        stop();
        forward(normSpeed,normSpeed);
        delay(300);
      }
    }
  }
  stop();
}

//dijkstra's shortest path

// A utility function to find the vertex with minimum distance value, from the set of vertices not yet included in shortest path tree
int minDistance(int dist[], bool sptSet[])
{
  // Initialize min value
  int min = INT_MAX, min_index;
  for (int v = 0; v < node_count; v++)
  {
    if (sptSet[v] == false && dist[v] <= min)
    {
      min = dist[v], min_index = v;
    }
  }
  return min_index;
}
 
// Function to save shortest path from source to destination
void savePath(int parent[], int destination, int src)
{
  for(int k = 0; k < node_count; ++k)
  {
    if(destination == src)
    {
      break;
    }
    PATH.insert(PATH.begin(), destination);
    destination = parent[destination];
  }
}
 
// Function that implements Dijkstra's single source shortest path algorithm for a graph represented using adjacency matrix representation
void dijkstra(int src, int destination)
{
  int dist[node_count];  // dist[i] will hold the shortest distance from src to i  
  bool sptSet[node_count]; // sptSet[i] will true if vertex i is included in shortest path tree or shortest distance from src to i is finalized
  int parent[node_count]; // Parent array to store shortest path tree

  // Initialize all distances as INFINITE and stpSet[] as false
  for (int i = 0; i < node_count; i++)
  {
    parent[0] = -1;
    dist[i] = INT_MAX;
    sptSet[i] = false;
  }
   
  dist[src] = 0; // Distance of source vertex from itself is always 0
 
    // Find shortest path for all vertices
  for (int count = 0; count < (node_count - 1); count++)
  {
    int u = minDistance(dist, sptSet);  // Pick the minimum distance vertex from the set of vertices not yet processed. u is always equal to src in first iteration.
    sptSet[u] = true; // Mark the picked vertex as processed

    // Update dist value of the adjacent vertices of the picked vertex.
    for (int v = 0; v < node_count; v++)
    {
      // Update dist[v] only if is not in sptSet, there is an edge from u to v, and total weight of path from src to v through u is smaller than current value of dist[v]
      if(!sptSet[v] && graph[u][v] && dist[u] + graph[u][v] < dist[v])
      {
        parent[v]  = u;
        dist[v] = dist[u] + graph[u][v];
      } 
    }         
  }
  
  savePath(parent, destination, src); // save the path
}
 
// to calculate new path, use this to find path
void driver(int src, int dest)
{
  PATH.clear();
  dijkstra(src, dest);
  PATH.insert(PATH.begin(), src);
} 

// to cut out a node
void sever_link(int node1, int node2)
{
  graph[node1][node2] = no_edge;
  graph[node2][node1] = no_edge;
}

void loop() 
{
  whichDirectionToGo();
  currentOrientation = nextOrientation;
  atThisNode = nextNode;
  
  //Main Control
  while(doneFlag != true)
  { 
    stop();
    forward((normSpeed), normSpeed);
    rightValue = rightLaser.readRange();
    leftValue = leftLaser.readRange();
    middleValue = middleLaser.readRange();
 
    // keep in middle of track
    if(rightValue < sideLowBound || leftValue > sideHighBound)
    {
      stop();
      forward(normSpeed, (1.25*normSpeed));
      delay(100); // was at 10
      
    }
    if(rightValue > sideHighBound || leftValue < sideLowBound)
    {
      stop();
      forward((1.25*normSpeed),normSpeed);
      delay(100); // was at 10
    }
    
    //if stuck on wall
    if(rightValue < crashSideDistance || leftValue < crashSideDistance)
    {
      while(rightValue < crashSideDistance || leftValue < crashSideDistance)
      {  
        stop();
        backward(normSpeed, normSpeed);
        delay(400);
        if(rightValue < crashSideDistance)
        {
          stop();
          left(normSpeed);
          delay(100);
          stop();
          forward(normSpeed,normSpeed);
          delay(300);
        }
        if(leftValue < crashSideDistance)
        {
          stop();
          right(normSpeed);
          delay(100);
          stop();
          forward(normSpeed,normSpeed);
          delay(300);
        }
        stop();
        rightValue = rightLaser.readRange();
        leftValue = leftLaser.readRange();
      }
    }

    if(atThisNode == destination && middleValue < frontDistance)
    {
      doneFlag = true;
      break;  
    }
    
    if(middleValue < frontDistance)
    {
      stop();
      backward(normSpeed,normSpeed);
      delay(300);
      stop();
      turnAround(normSpeed);
      delay(1000);
      sever_link(previousNode,atThisNode);
      atThisNode = previousNode;
      if(currentOrientation = 'N')
      {
        currentOrientation = 'S';
      }
      else if(currentOrientation = 'S')
      {
        currentOrientation = 'N';
      }
      else if(currentOrientation = 'W')
      {
        currentOrientation = 'E';
      }
      else if(currentOrientation = 'E')
      {
        currentOrientation = 'W';
      }
    }
    
    if(rightLaser.readRange() == infiniteDistance)
    {
      delay(350);
      stop();
      delay(100); 
      whichDirectionToGo();
      atThisNode = nextNode;
      currentOrientation = nextOrientation;
    }

    if(leftLaser.readRange() == infiniteDistance)
    {
      delay(350);
      stop();
      delay(100);
      whichDirectionToGo();
      atThisNode = nextNode;
      currentOrientation = nextOrientation;
    }

  }
  
  while(true)
  {
    stop();
    delay(10000); 
  }
  
}

/*
  // Testing lasers
  Serial.println("Right Laser: ");
  Serial.println(rightLaser.readRange());
  delay(1000);
  Serial.println("Middle Laser: ");
  Serial.println(middleLaser.readRange());
  delay(1000);
  Serial.println("Left Laser: ");
  Serial.println(leftLaser.readRange());
  delay(1000);

*/ 


