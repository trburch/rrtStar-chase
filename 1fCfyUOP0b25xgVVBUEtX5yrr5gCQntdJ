#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <sys/types.h>
#include <time.h>
#include <limits>
#include <sys/time.h>
#include <math.h>
#include <iostream>
#include <typeinfo>
using namespace std;

#include "graph_proj.h"
const double PI = 3.141592653589793238463;
double LARGE = std::numeric_limits<double>::infinity();
// returs a random double between 0 and 1
double rand_d(){
  return (double)rand() / (double)RAND_MAX;
}

// returns the euclidian distance between two nodes
double euDist(Node* node1, Node* node2){
  return sqrt(pow(node1->x - node2->x,2)+pow(node1->y - node2->y,2)+pow(node1->z - node2->z,2));
}

Node* NN(Node* sampleNode, Graph &G){
  Node* nearestNode = &G.nodes[0];
  Node* testNode;
  double dist;
  for(int ii = 0; ii < G.numNodes; ii++)
  {
    testNode = &G.nodes[ii];
    if(ii == 0){
      dist = euDist(sampleNode, testNode);
    } else if(euDist(sampleNode, testNode) < dist){
      dist = euDist(sampleNode, testNode);
      nearestNode = testNode;
    }
  }
  return nearestNode;
}

void rNN(Node* stepNode, Graph &G, double rBall){
  Node* testNode;
  stepNode->neighborNodes = (Node**)calloc(200,sizeof(Node*));
  int numNeighbors = 0;
  for(int ii = 0; ii < G.numNodes; ii++)
  {
    //cout<< ii<< endl;
    testNode = &G.nodes[ii];
    if(euDist(stepNode, testNode) < rBall){
      stepNode->neighborNodes[numNeighbors] = testNode;
      numNeighbors++;
      //cout<< "Inserting Node -> Radius:"<<euDist(stepNode, testNode)<< "<"<< rBall<< endl;
    }
  }
  cout<< "Number of neighbor Nodes:"<<numNeighbors<< endl;
  return;
}

double rParam(Graph &G,double xDim, double yDim, double zDim){
  double Xfree = xDim*yDim*zDim;   // not subtracting obstacles because of complexity of intersections
                                   // this will provide a conservative estimate of gamPRM
  double gamPRM = 2.0*pow(1.0+(1.0/3.0),1.0/3.0)*pow(Xfree/(4.0*PI/3.0),1.0/3.0);
  double rBall = gamPRM*pow(log10(G.numNodes)/G.numNodes,1.0/3.0);
  return rBall;
}

void geodesic(Node* nearestNode, Node* samplePoint, double geo[]){
  double x, y, z;
  x = samplePoint->x - nearestNode->x;
  y = samplePoint->y - nearestNode->y;
  z = samplePoint->z - nearestNode->z;
  double psi = atan2(y,x);
  double theta = asin(z/euDist(nearestNode,samplePoint));
  geo[0] = psi; geo[1] = theta;
  //cout<<"The geodesic inside the function is: ("<<psi<<","<<theta<<")"<<endl;
  return;
}

//aproximates robot as sphere to account for any attitude
bool collision(Node* stepNode, Graph &G, double epsilon, double delta){
  int pathSteps = floor(epsilon/delta)+1;
  Node* testObstacle;
  Node* robotNode;
  Node* testNode = (Node*)calloc(1,sizeof(Node));
  for(int ii = 0; ii < G.numObstacles; ii++)
  {
    testObstacle = &G.obstacles[ii];
    for(int jj = 0; jj < pathSteps; jj++)
    {
      // projects the robot back along its path at delta sized steps
      testNode->x = stepNode->x - (delta*jj)*cos(stepNode->theta)*cos(stepNode->psi);
      testNode->y = stepNode->y - (delta*jj)*cos(stepNode->theta)*sin(stepNode->psi);
      testNode->z = stepNode->z - (delta*jj)*sin(stepNode->theta);

      if(euDist(testNode, testObstacle) <= testObstacle->radius + G.robotRadius){
        //cout<< "I hit or am too close for comfort to an obstacle"<< endl;
        return true;
      }else if(testNode->x<=0||testNode->x>=100||testNode->y<=0||testNode->y>=100||testNode->z<=0||testNode->z>=100){
        //cout<< "I am out of bounds"<< endl;
        return true;
      }else{
        //cout<<"I'm all good, check the next obstacle"<<endl;
      }
    }
  }
  //cout<< "No collisions"<< endl;
  return false;
}

int main(){

  srand(time(NULL)); // seed random number generator
  //const double PI = 3.141592653589793238463;

  Graph G;
  G.readObstaclesFromFile("obstacles_proj.txt");
  cout << "Number of obstacles is: "<< G.numObstacles << endl;

  G.readRobotFromFile("proj_robot.txt");
  cout << "Number of points in the Robot is: "<< G.numRobotPts << endl;

  // intitialize the node array
  int maxNodes = 10000;
  int maxSamples = 10000000;
  G.nodes = (Node*)calloc(maxNodes,sizeof(Node));
  G.goal  = (Node*)calloc(1,sizeof(Node));

  // place goal and start in the node array (goal has index of 0 and start has index of 1)
  int goalNodeIndex = 0;
  int startNodeIndex = 0;
  Node* goalNode = &G.goal[goalNodeIndex];
  Node* startNode = &G.nodes[startNodeIndex];
  G.numNodes = 1;

  //The max dimensions of the space being sampled
  double xDim = 100; double yDim = 100; double zDim = 100;
  double psiDim = 2*PI; double thetaDim = PI;

  // we want to find a path that goes from here to here
  startNode->x = 60; startNode->y = 60; startNode->z = 20; startNode->psi = PI/4;
  goalNode->x = 0; goalNode->y = 0; goalNode->z = 0; goalNode->radius = 20;
  cout << "Start is at: ("<<startNode->x<<","<<startNode->y<<","<<startNode->z<<")" << endl;
  cout << "Goal is at: ("<<goalNode->x<<","<<goalNode->y<<","<<goalNode->z<<")"<< endl;

  double epsilon = 2;  // step size
  double delta = 0.25;  // collision resolution

  // samples the goal every goalBias iterations
  int goalBias = 2000;

  bool goalFound = false;

  Node tempNode; Node newNode;
  Node* samplePoint = &tempNode;
  Node* stepNode = &newNode;
  Node* nearestNode;
  Node* robotNode;
  int numSamples = 0;
  double rBall = 0;
  double geo[] = {0,0};
  while(!goalFound && G.numNodes < maxNodes && numSamples < maxSamples){
    //cout<<"Number of nodes in Array: "<<G.numNodes<<endl;
    if(numSamples % goalBias == 0){
      samplePoint->x = goalNode->x; samplePoint->y = goalNode->y; samplePoint->z = goalNode->z;
      //cout<<"Sampling the goal"<< endl;
    }else{
      samplePoint->x = rand_d()*xDim; samplePoint->y = rand_d()*yDim; samplePoint->z = rand_d()*zDim; // select sample point at random
      //cout<<"Taking Random Sample at: ("<<samplePoint->x<<","<<samplePoint->y<<","<<samplePoint->z<<")"<< endl;
    }
    numSamples++;
    nearestNode = NN(samplePoint, G);  // find nearest neighbor
    //cout<< "Found nearest neighbor"<<endl;
    //stepPoint->psi = geodesic(nearestNode, samplePoint); //geodesic
    geodesic(nearestNode, samplePoint, geo);
    //cout<<"The geodesic is: ("<<geo[0]<<","<<geo[1]<<")"<<endl;
    stepNode->psi = geo[0];
    stepNode->theta = geo[1];

    // step out epsilon amount from neighbor
    stepNode->x = nearestNode->x + epsilon*cos(stepNode->theta)*cos(stepNode->psi);
    stepNode->y = nearestNode->y + epsilon*cos(stepNode->theta)*sin(stepNode->psi);
    stepNode->z = nearestNode->z + epsilon*sin(stepNode->theta);

    //collision check
    if(collision(stepNode, G, epsilon, delta)? 0 : 1){
      //cout<< "Line from: ("<<nearestNode->x  << "," << nearestNode->y  << ")" <<endl;
      //cout<< "Inserting New Node at: ("  << stepNode->x  << "," << stepNode->y  << ")" <<endl;
      //cout <<"Distance to goal is: "<<euDist(stepNode,goalNode)<<endl;
      rBall = rParam(G, xDim, yDim, zDim);
      cout<<"The radius of the ball is:"<<rBall<<endl;
      rNN(stepNode, G, rBall);  // add neighbors into neighborNode array
      cout<< "I'm out"<< endl;

      newNode.parentNode = nearestNode;
      newNode.id = G.numNodes;
      G.nodes[G.numNodes] = newNode; //this is the node pointed to by stepNode
      G.numNodes = G.numNodes+1;

      if(euDist(stepNode,goalNode) < goalNode->radius){
        goalFound = true;
        cout<<"Found the Goal!"<<endl;
        //newNode.parentNode = nearestNode;
        //newNode.id = numNodes;
        G.savePathToFile("output_path.txt", stepNode);
        break;
      }
    }
  }
  if(goalFound == false) cout<<"Could not find Goal"<<endl;
  G.saveSearchTreeToFile("search_tree.txt");
  G.saveProblemToFile("setup.txt");
  return 0;
}
