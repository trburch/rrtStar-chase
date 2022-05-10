#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <sys/types.h>
#include <time.h>
#include <sys/time.h>
#include <math.h>
#include <limits>
#include <iostream>
#include <typeinfo>
#include <vector>
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

double rParam(Graph &G,double xDim, double yDim, double zDim){
  double Xfree = xDim*yDim*zDim;   // not subtracting obstacles because of complexity of intersections
                                   // this will provide a conservative estimate of gamPRM
  double fudgeFactor = 0.5;
  double gamPRM = 2.0*pow(1.0+(1.0/3.0),1.0/3.0)*pow(Xfree/(4.0*PI/3.0),1.0/3.0);
  double rBall = 0;
  if(G.numNodes == 1){
    rBall = sqrt(2.0)*100;
  }else{
    rBall = fudgeFactor*gamPRM*pow(log10(G.numNodes)/G.numNodes,1.0/3.0);
  }
  //cout<<"The radius of the ball is:"<<rBall<<endl;
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
bool simple_collision(Node* startNode, Node* endNode, Graph &G, double delta){

  double x, y, z;
  x = endNode->x - startNode->x;
  y = endNode->y - startNode->y;
  z = endNode->z - startNode->z;
  double psi = atan2(y,x);
  double theta = asin(z/euDist(startNode,endNode));

  int pathSteps = floor(euDist(startNode,endNode)/delta)+1;
  Node* testObstacle;
  Node* testNode = (Node*)calloc(1,sizeof(Node));
  for(int ii = 0; ii < G.numObstacles; ii++)
  {
    testObstacle = &G.obstacles[ii];
    for(int jj = 0; jj < pathSteps; jj++)
    {
      // projects the robot back along its path at delta sized steps
      testNode->x = endNode->x - (delta*jj)*cos(theta)*cos(psi);
      testNode->y = endNode->y - (delta*jj)*cos(theta)*sin(psi);
      testNode->z = endNode->z - (delta*jj)*sin(theta);

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

void rNN(Node* stepNode, Graph &G, double rBall, double delta){
  Node* testNode;
  int numNeighbors = 0;
  for(int ii = 0; ii < G.numNodes; ii++)
  {
    testNode = &G.nodes[ii];
    if(euDist(stepNode, testNode) < rBall && !simple_collision(stepNode, testNode, G, delta)){
      //cout<<testNode<<endl;
      stepNode->neighborNodes.push_back(testNode);
      testNode->neighborNodes.push_back(stepNode);
    }
  }
  //cout<<"Number of Neighbors: "<< stepNode->neighborNodes.size()<< endl;
  //getchar();
  return;
}

void newNodeCost(Node* stepNode, Graph &G){
  //initialize cost value to the value of stepping form the first neighbor
  stepNode->costToNode = stepNode->neighborNodes[0]->costToNode + euDist(stepNode,stepNode->neighborNodes[0]);
  //initialize the parent as the first neigbor
  stepNode->parentNode = stepNode->neighborNodes[0];
  Node* testNode;
  //find the minimum cost neighbor (accounting for the edge length to stepNode)
  for(int ii = 0; ii < stepNode->neighborNodes.size(); ii++){
    testNode = stepNode->neighborNodes[ii];
    //cout<<"Cost to "<<testNode<<" is: "<<testNode->costToNode + euDist(stepNode,testNode)<<endl;
    if(testNode->costToNode + euDist(stepNode,testNode) < stepNode->costToNode){

      stepNode->costToNode = testNode->costToNode + euDist(stepNode,testNode);
      stepNode->parentNode = testNode;
    }
  }
  //cout<< "Parent should be "<<testNode<< endl;
  //cout<<stepNode->parentNode<<" is the parent node of "<< stepNode<< endl;

  return;
}

void rewireAndPropagate(Node* stepNode, Graph &G){
    Node* testNode;
    // re-wires local Nodes in RRT*
    for(int ii = 0; ii < stepNode->neighborNodes.size(); ii++){
      testNode = stepNode->neighborNodes[ii];
      if(stepNode->costToNode + euDist(stepNode,testNode) < testNode->costToNode && testNode != stepNode->parentNode){
        //cout<<"Rewiring Node "<<testNode->id<< endl;
        testNode->costToNode = stepNode->costToNode + euDist(stepNode,testNode);
        testNode->parentNode = stepNode;
        //getchar();
      }
    }

    // vector<Node*> brokenNodes;
    // while(){
    //   for(int ii = 0; ii < thisNode->neighborNodes.size(); ii++){
    //     testNode = thisNode->neighborNodes[ii];
    //     if(thisNode->costToNode + euDist(thisNode,testNode) < testNode->costToNode && testNode != thisNode->parentNode){
    //       brokenNodes.push_back(testNode)
    //     }
    //   }
    //   thisNode = brokenNodes.front()
    // }


  return;
}

// void expand(Heap<Node> &H, Node* thisNode){
//   for(int n = 0; n < thisNode->neighborNodes.size(); n++)
//   {
//     Node* rewireNode = stepNode->neighborNodes[ii]; // pointer to the neighbor node being considered
//
//     if(rewireNode->status == 0 || rewireNode->costToNode > thisNode->costToNode + euDist(rewireNode, thisNode))  // neighbor has not yet been visited or has greater cost than current path
//     {
//       rewireNode->costToNode = thisNode->costToNode + euDist(rewireNode, thisNode); // Path cost to Neighbor
//       double rewireKey = rewireNode->costToNode; // Path cost
//       H.addToHeap(rewireNode, rewireKey);
//
//       // remeber this node as its parent
//       rewireNode->parentNode = thisNode;
//
//       // make sure it is in the open list
//       rewireNode->status = 1;
//     }
//   }
//
//   thisNode->status = 2;    // now this node is in the closed list
// }

int main(){

  srand(time(NULL)); // seed random number generator
  //const double PI = 3.141592653589793238463;

  Graph G;
  G.readObstaclesFromFile("obstacles_proj.txt");
  cout << "Number of obstacles is: "<< G.numObstacles << endl;

  G.readRobotFromFile("proj_robot.txt");
  cout << "Number of points in the Robot is: "<< G.numRobotPts << endl;

  // intitialize the node array
  int maxNodes = 2000;
  int maxSamples = 1000000;
  G.nodes = (Node*)calloc(maxNodes,sizeof(Node));
  G.goal  = (Node*)calloc(1,sizeof(Node));

  // place goal and start in the node array
  int goalNodeIndex = 0;
  int startNodeIndex = 0;
  Node* goalNode = &G.goal[goalNodeIndex]; goalNode->costToNode = LARGE;
  Node* startNode = &G.nodes[startNodeIndex]; startNode->costToNode = 0.0;
  G.numNodes = 1;

  //The max dimensions of the space being sampled
  double xDim = 100; double yDim = 100; double zDim = 100;
  double psiDim = 2*PI; double thetaDim = PI;

  // we want to find a path that goes from here to here
  startNode->x = 60; startNode->y = 60; startNode->z = 20; startNode->psi = PI/4;
  goalNode->x = 0; goalNode->y = 0; goalNode->z = 0; goalNode->radius = 20;
  cout << "Start is at: ("<<startNode->x<<","<<startNode->y<<","<<startNode->z<<")" << endl;
  cout << "Goal is at: ("<<goalNode->x<<","<<goalNode->y<<","<<goalNode->z<<")"<< endl;

  double epsilon = 5;  // step size
  double delta = 0.25;  // collision resolution

  // samples the goal every goalBias iterations
  int goalBias = 1000;

  bool goalFound = false;

  Node tempNode; Node newNode;
  Node* samplePoint = &tempNode;
  Node* stepNode = &newNode;
  Node* nearestNode;
  Node* robotNode;
  int numSamples = 0;
  double rBall = 0;
  double geo[] = {0,0};
  //while(!goalFound && G.numNodes < maxNodes && numSamples < maxSamples){ //RRT
  while(G.numNodes < maxNodes && numSamples < maxSamples){ //RRT* and RRT#
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

    if(euDist(nearestNode, samplePoint) > epsilon){
      geodesic(nearestNode, samplePoint, geo);
      //cout<<"The geodesic is: ("<<geo[0]<<","<<geo[1]<<")"<<endl;
      stepNode->psi = geo[0];
      stepNode->theta = geo[1];

      // step out epsilon amount from neighbor
      stepNode->x = nearestNode->x + epsilon*cos(stepNode->theta)*cos(stepNode->psi);
      stepNode->y = nearestNode->y + epsilon*cos(stepNode->theta)*sin(stepNode->psi);
      stepNode->z = nearestNode->z + epsilon*sin(stepNode->theta);
    }else{
      geodesic(nearestNode, samplePoint, geo);
      //cout<<"The geodesic is: ("<<geo[0]<<","<<geo[1]<<")"<<endl;
      stepNode->psi = geo[0];
      stepNode->theta = geo[1];

      stepNode->x = samplePoint->x;
      stepNode->y = samplePoint->y;
      stepNode->z = samplePoint->z;
    }


    //collision check
    if(simple_collision(nearestNode, stepNode, G, delta)? 0 : 1){
      //cout<< "Line from: ("<<nearestNode->x  << "," << nearestNode->y  << ")" <<endl;
      //cout<< "Inserting New Node at: ("  << stepNode->x  << "," << stepNode->y  << ")" <<endl;
      //cout <<"Distance to goal is: "<<euDist(stepNode,goalNode)<<endl;
      rBall = rParam(G, xDim, yDim, zDim);
      rNN(stepNode, G, rBall, delta);  // add neighbors into neighborNode vector
      newNodeCost(stepNode, G); //assigns parent of stepNode

      newNode.id = G.numNodes;
      G.nodes[G.numNodes] = newNode; //this is the node pointed to by stepNode
      rewireAndPropagate(&G.nodes[G.numNodes], G);

      if(euDist(stepNode,goalNode) < goalNode->radius && stepNode->costToNode+euDist(stepNode,goalNode)<goalNode->costToNode){
        goalFound = true;
        goalNode->parentNode = &G.nodes[G.numNodes];
        goalNode->costToNode = stepNode->costToNode+euDist(stepNode,goalNode);
        cout<<"Found the Goal!"<<endl;
        //break;
      }
      cout<<"Cost to Goal is: "<< goalNode->costToNode <<endl;
      stepNode->neighborNodes.clear();
      G.numNodes = G.numNodes+1;
    }
  }
  if(goalFound == false){
    cout<<"Could not find Goal"<<endl;
  }else{
    G.savePathToFile("output_path.txt", goalNode);
  }
  G.saveSearchTreeToFile("search_tree.txt");
  G.saveProblemToFile("setup.txt");
  return 0;
}
