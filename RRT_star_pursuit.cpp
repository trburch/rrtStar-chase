#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <sys/types.h>
#include <time.h>
#include <sys/time.h>
#include <limits>
#include <math.h>
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

// cross product of two vector array.
void crossProduct(Node* ptr1, Node* ptr2, Node* tempPtr){
    tempPtr->x = ptr1->y * ptr2->z - ptr1->z * ptr2->y;
    tempPtr->y = ptr1->x * ptr2->z - ptr1->x * ptr2->x;
    tempPtr->z = ptr1->x * ptr2->y - ptr1->y * ptr2->x;
    return;
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
  return;
}

bool RRT_star(Graph &G, Node* agentPtr, Node* evaderPtr, double captureRad, double* trajectory, double agentVel, double targetDist){

  srand(time(NULL)); // seed random number generator

  // intitialize the node array
  int maxNodes = floor(15*targetDist);
  int maxSamples = 100000;
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
  startNode->x = agentPtr->x; startNode->y = agentPtr->y; startNode->z = agentPtr->z; startNode->psi = 0.0*PI/4;
  goalNode->x = evaderPtr->x; goalNode->y = evaderPtr->y; goalNode->z = evaderPtr->z; goalNode->radius = captureRad;

  double epsilon = agentVel;  // step size
  double delta = 0.25;  // collision resolution

  // samples the goal every goalBias iterations
  int goalBias = 200;

  bool goalFound = false;

  Node tempNode; Node newNode;
  Node* samplePoint = &tempNode;
  Node* stepNode = &newNode;
  Node* nearestNode;
  Node* thisNode;
  int numSamples = 0;
  double rBall = 0;
  double geo[] = {0,0};

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
        //cout<<"Found a path!"<<endl;
        //break;
      }
      //cout<<"Cost to Goal is: "<< goalNode->costToNode <<endl;
      stepNode->neighborNodes.clear();
      G.numNodes = G.numNodes+1;
    }
  }
  if(goalFound == false){
    cout<<"Could not find a path"<<endl<<"Agent is staying still"<< endl;
    trajectory[0] = 0; trajectory[1] = 0;
    return false;
  }else{
    cout<<"Path cost to evader is: "<< goalNode->costToNode <<endl;
    // alters trajectory
    thisNode = goalNode;
    while(thisNode->parentNode != startNode)
    {
      thisNode = thisNode->parentNode;
    }
    geodesic(startNode,thisNode,trajectory);
    return true;
  }
}

bool RRT_star_evader(Graph &G, Node* evaderPtr, Node* evaderGoal, Node* agent1Ptr, Node* agent2Ptr,double captureRad, double* trajectory, double evaderVel){

  srand(time(NULL)); // seed random number generator

  // intitialize the node array
  int maxNodes = floor(30*euDist(evaderPtr,evaderGoal));
  int maxSamples = 100000;
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
  startNode->x = evaderPtr->x; startNode->y = evaderPtr->y; startNode->z = evaderPtr->z; startNode->psi = 0.0*PI/4;
  goalNode->x = evaderGoal->x; goalNode->y = evaderGoal->y; goalNode->z = evaderGoal->z; goalNode->radius = captureRad;
  if(euDist(startNode,goalNode)< captureRad){
    cout<<"Evader is where it wants to be"<<endl<<"Evader is staying still"<< endl;
    trajectory[0] = 0; trajectory[1] = 0;
    return false;
  }

  double epsilon = evaderVel;  // step size
  double delta = 0.25;  // collision resolution

  // samples the goal every goalBias iterations
  int goalBias = 200;

  bool goalFound = false;

  Node tempNode; Node newNode;
  Node* samplePoint = &tempNode;
  Node* stepNode = &newNode;
  Node* nearestNode;
  Node* thisNode;
  int numSamples = 0;
  double rBall = 0;
  double geo[] = {0,0};

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
    if(!simple_collision(nearestNode, stepNode, G, delta) && euDist(stepNode, agent1Ptr)> evaderVel+captureRad && euDist(stepNode, agent2Ptr)> evaderVel+captureRad){
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
        //cout<<"Found a path!"<<endl;
        //break;
      }
      //cout<<"Cost to Goal is: "<< goalNode->costToNode <<endl;
      stepNode->neighborNodes.clear();
      G.numNodes = G.numNodes+1;
    }
  }
  if(goalFound == false){
    cout<<"Could not find a path"<<endl<<"Evader is staying still"<< endl;
    trajectory[0] = 0; trajectory[1] = 0;
    return false;
  }else{
    cout<<"Path cost to open space is: "<< goalNode->costToNode <<endl;
    // alters trajectory
    thisNode = goalNode;
    while(thisNode->parentNode != startNode)
    {
      thisNode = thisNode->parentNode;
    }
    geodesic(startNode,thisNode,trajectory);
    return true;
  }
}

void evaderStepAway(Graph &G, Node* evaderPtr, Node* agent1Ptr, Node* agent2Ptr, double evaderVel){
  Node newNode; Node* stepNode = &newNode;
  int numSamples = 0; int maxSamples = 1000;
  double geo1[] = {0,0};
  double geo2[] = {0,0};
  double geoCent[] = {0,0};
  bool safeStep = false;
  Node center; Node* centerPtr = &center;
  center.x = 50; center.y = 50; center.z = 50;
  while(!safeStep && numSamples < maxSamples){
    if(numSamples == 0){
      geodesic(evaderPtr, agent1Ptr, geo1);
      geodesic(evaderPtr, agent2Ptr, geo2);
      stepNode->psi = ((geo1[0]+geo2[0])/2.0)+PI;
      stepNode->theta = -((geo1[1]+geo2[1])/2.0);
      //cout<<"Moving directly away from agents position average"<< endl;
    }else{
      //cout<<"Moving in Random Direction"<< endl;
      stepNode->psi = rand_d()*2*PI;
      stepNode->theta = rand_d()*2*PI;
    }
    numSamples++;
    //cout<< numSamples<< endl;

    // step out epsilon amount from current position
    stepNode->x = evaderPtr->x + evaderVel*cos(stepNode->theta)*cos(stepNode->psi);
    stepNode->y = evaderPtr->y + evaderVel*cos(stepNode->theta)*sin(stepNode->psi);
    stepNode->z = evaderPtr->z + evaderVel*sin(stepNode->theta);

    if(simple_collision(evaderPtr, stepNode, G, 0.25)? 0 : 1){
      safeStep = true;
      evaderPtr->x = stepNode->x;
      evaderPtr->y = stepNode->y;
      evaderPtr->z = stepNode->z;
      return;
    }
  }
}

void evaderStepTangent(Graph &G, Node* evaderPtr, Node* agent1Ptr, Node* agent2Ptr, double evaderVel){
  Node newNode; Node* stepNode = &newNode;
  int numSamples = 0; int maxSamples = 1000;
  double geo[] = {0,0};
  bool safeStep = false;
  while(!safeStep && numSamples < maxSamples){
    if(numSamples == 0){
      crossProduct(agent1Ptr, agent2Ptr, stepNode);
      geodesic(evaderPtr, stepNode, geo);
      stepNode->psi = geo[0]+PI;
      stepNode->theta = -geo[1];
      //cout<<"Moving directly away from agents position average"<< endl;
    }else{
      //cout<<"Moving in Random Direction"<< endl;
      stepNode->psi = rand_d()*2*PI;
      stepNode->theta = rand_d()*2*PI;
    }
    numSamples++;
    //cout<< numSamples<< endl;

    // step out epsilon amount from current position
    stepNode->x = evaderPtr->x + evaderVel*cos(stepNode->theta)*cos(stepNode->psi);
    stepNode->y = evaderPtr->y + evaderVel*cos(stepNode->theta)*sin(stepNode->psi);
    stepNode->z = evaderPtr->z + evaderVel*sin(stepNode->theta);

    if(simple_collision(evaderPtr, stepNode, G, 0.25)? 0 : 1){
      safeStep = true;
      evaderPtr->x = stepNode->x;
      evaderPtr->y = stepNode->y;
      evaderPtr->z = stepNode->z;
      return;
    }
  }
}

void evaderOpenSpace(Graph &G, Node* evaderPtr, Node* agent1Ptr, Node* agent2Ptr, Node* evaderGoal, double evaderVel, double captureRad){
  Node tempNode; Node* samplePoint = &tempNode;
  double agent1Dist = euDist(evaderGoal, agent1Ptr); double agent2Dist = euDist(evaderGoal, agent2Ptr);
  bool sampleSafe = true;
   bool evaderMove = false;
  for(int jj = 0; jj < 1500; jj++){
    samplePoint->x = rand_d()*100; samplePoint->y = rand_d()*100; samplePoint->z = rand_d()*100;
    sampleSafe = true;
    //check if sample is in an obstacle
    Node* testObstacle;
    for(int ii = 0; ii < G.numObstacles; ii++)
    {
      testObstacle = &G.obstacles[ii];
      if(euDist(samplePoint, testObstacle) <= testObstacle->radius + G.robotRadius){
        //cout<< "I hit an obstacle or am too close for comfort"<< endl;
        sampleSafe = false;
      }else if(samplePoint->x<=0||samplePoint->x>=100||samplePoint->y<=0||samplePoint->y>=100||samplePoint->z<=0||samplePoint->z>=100){
        //cout<< "I am out of bounds"<< endl;
        sampleSafe = false;
      }else{
        //cout<<"I'm all good, check the next obstacle"<<endl;
      }
    }
    // sample that is farthest from the agents is the goal
    if(sampleSafe && euDist(samplePoint, agent1Ptr)> agent1Dist && euDist(samplePoint, agent2Ptr)> agent2Dist ){
      agent1Dist = euDist(samplePoint, agent1Ptr);
      agent2Dist = euDist(samplePoint, agent2Ptr);
      evaderGoal->x = samplePoint->x;
      evaderGoal->y = samplePoint->y;
      evaderGoal->z = samplePoint->z;
      //cout<<"("<<evaderGoal->x<<","<<evaderGoal->y<<","<<evaderGoal->z<<")"<<endl;
      //getchar();
    }
  }
  double evaderTraj[] = {0,0};
  evaderMove = RRT_star_evader(G, evaderPtr, evaderGoal, agent1Ptr, agent2Ptr, captureRad, evaderTraj ,evaderVel);
  //cout<< "Made it past RRT*"<< endl;
  if(evaderMove){
    evaderPtr->x = evaderPtr->x + evaderVel*cos(evaderTraj[1])*cos(evaderTraj[0]);
    evaderPtr->y = evaderPtr->y + evaderVel*cos(evaderTraj[1])*sin(evaderTraj[0]);
    evaderPtr->z = evaderPtr->z + evaderVel*sin(evaderTraj[1]);
  }
  return;
}

int main(){

  Graph G;
  G.readObstaclesFromFile("obstacles_proj.txt");
  cout << "Number of obstacles is: "<< G.numObstacles << endl;

  G.readRobotFromFile("proj_robot.txt");
  cout << "Number of points in the Robot is: "<< G.numRobotPts << endl;

  FILE * a1File = fopen("Experiments\\agent1_path_112.txt","w");
  FILE * a2File = fopen("Experiments\\agent2_path_112.txt","w");
  FILE * eFile  = fopen("Experiments\\evader_path_112.txt","w");

  Node agent1; Node* agent1Ptr = &agent1;
  agent1Ptr->x = 90; agent1Ptr->y = 10; agent1Ptr->z = 90;
  double agent1Traj[] = {0, 0}; double agent1Vel = 5;
  double target1Dist = LARGE; bool agent1Move = false;

  Node agent2; Node* agent2Ptr = &agent2;
  agent2Ptr->x = 10; agent2Ptr->y = 80; agent2Ptr->z = 10;
  double agent2Traj[] = {0, 0}; double agent2Vel = 5;
  double target2Dist = LARGE; bool agent2Move = false;

  Node evader; Node* evaderPtr = &evader;
  Node evaderGoalNode; Node* evaderGoal = &evaderGoalNode;
  evaderPtr->x = 50; evaderPtr->y = 50; evaderPtr->z = 20;
  evaderGoal->x = evaderPtr->x; evaderGoal->y = evaderPtr->y; evaderGoal->z = evaderPtr->z;
  double evaderVel = 12;

  double captureRad = 10;
  bool captureFlag = false;
  int simTime = 0;
  while(!captureFlag){
    // writes the positions of the agent and the evader to a file
    // uses +1 indexing for Matlab
    fprintf(a1File, "%d, %f, %f, %f\n",
            simTime+1, agent1Ptr->x, agent1Ptr->y, agent1Ptr->z);
    fprintf(a2File, "%d, %f, %f, %f\n",
            simTime+1, agent2Ptr->x, agent2Ptr->y, agent2Ptr->z);
    fprintf(eFile, "%d, %f, %f, %f\n",
            simTime+1, evaderPtr->x, evaderPtr->y, evaderPtr->z);

    target1Dist = euDist(agent1Ptr,evaderPtr);
    target2Dist = euDist(agent2Ptr,evaderPtr);
    if(target1Dist<captureRad || target2Dist<captureRad){
      captureFlag = true;
      cout << "Agent 1 is at: ("<<agent1Ptr->x<<","<<agent1Ptr->y<<","<<agent1Ptr->z<<")" << endl;
      cout << "Agent 2 is at: ("<<agent2Ptr->x<<","<<agent2Ptr->y<<","<<agent2Ptr->z<<")" << endl;
      cout << "Evader is at: ("<<evaderPtr->x<<","<<evaderPtr->y<<","<<evaderPtr->z<<")"<< endl;
      if(target1Dist<captureRad){
        cout<< "Evader Captured by Agent 1!"<< endl;
      }else if(target2Dist<captureRad){
        cout<< "Evader Captured by Agent 2!"<< endl;
      }
      cout<< "Capture took "<< simTime << " seconds"<< endl;

      fclose(a1File);
      fclose(a2File);
      fclose(eFile);
      break;
    }else if(simTime>=120){
      cout<<"EVADER AVOIDED CAPTURE!"<<endl;
      fclose(a1File);
      fclose(a2File);
      fclose(eFile);
      break;
    }

    cout<< endl;
    cout << "Time: "<< simTime<< endl;
    cout << "Agent 1 is at: ("<<agent1Ptr->x<<","<<agent1Ptr->y<<","<<agent1Ptr->z<<")" << endl;
    cout << "Agent 2 is at: ("<<agent2Ptr->x<<","<<agent2Ptr->y<<","<<agent2Ptr->z<<")" << endl;
    cout << "Evader is at: ("<<evaderPtr->x<<","<<evaderPtr->y<<","<<evaderPtr->z<<")"<< endl;

    cout<< "Calculating Agent 1 path..."<<endl;
    agent1Move = RRT_star(G, agent1Ptr, evaderPtr, captureRad, agent1Traj, agent1Vel, target1Dist); //calculates viable path to the evader
    //cout<<"The trajectory of Agent 1 is: ("<<agent1Traj[0]<<","<<agent1Traj[1]<<")"<<endl;
    G.nodes = NULL; //clears nodes from graph for next iteration

    cout<< "Calculating Agent 2 path..."<<endl;
    agent2Move = RRT_star(G, agent2Ptr, evaderPtr, captureRad, agent2Traj, agent2Vel, target2Dist); //calculates viable path to the evader
    //cout<<"The trajectory of Agent 2 is: ("<<agent2Traj[0]<<","<<agent2Traj[1]<<")"<<endl;
    G.nodes = NULL; //clears nodes from graph for next iteration

    cout<< "Calculating evader step..."<<endl;
    //evaderStepAway(G,evaderPtr,agent1Ptr, agent2Ptr, evaderVel); //(MUST COME AFTER AGENTS PATHS)
    //evaderStepTangent(G,evaderPtr,agent1Ptr, agent2Ptr, evaderVel); //(MUST COME AFTER AGENTS PATHS)
    evaderOpenSpace(G, evaderPtr, agent1Ptr, agent2Ptr, evaderGoal, evaderVel, captureRad); //(MUST COME AFTER AGENTS PATHS)
    //cout<< "The evader has moved"<<endl;
    G.nodes = NULL;
    G.goal = NULL;

    // Update Agent 1 Position (MUST BE AFTER ALL PATHS ARE CALCULATED)
    if(agent1Move){
      agent1Ptr->x = agent1Ptr->x + agent1Vel*cos(agent1Traj[1])*cos(agent1Traj[0]);
      agent1Ptr->y = agent1Ptr->y + agent1Vel*cos(agent1Traj[1])*sin(agent1Traj[0]);
      agent1Ptr->z = agent1Ptr->z + agent1Vel*sin(agent1Traj[1]);
    }

    // Update Agent 2 Position (MUST BE AFTER ALL PATHS ARE CALCULATED)
    if(agent2Move){
      agent2Ptr->x = agent2Ptr->x + agent2Vel*cos(agent2Traj[1])*cos(agent2Traj[0]);
      agent2Ptr->y = agent2Ptr->y + agent2Vel*cos(agent2Traj[1])*sin(agent2Traj[0]);
      agent2Ptr->z = agent2Ptr->z + agent2Vel*sin(agent2Traj[1]);
    }

    simTime++;

  }


  return 0;
}
