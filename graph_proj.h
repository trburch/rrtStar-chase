/*  Copyright Michael Otte, 2018
 *
Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 *  Basic random graph with nodes and edges
 *
 *  Note: that this reads nodes from file with 1-based indexing and
 *  switches node IDs to use 0-based indexing. In other words,
 *  nodes from a file have their IDs reduced by 1 within this code.
 */
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <sys/types.h>
#include <time.h>
#include <math.h>
#include <vector>


struct Node;        // NOTE: we'll define this in detail later, but we need it now


// make the basic edge data stucture
struct Edge
{
  Node* startNode;     // the edge starts at this node
  Node* endNode;       // the edge goes to this node

  double edgeCost;  // going along the edge cost this much
};


// this is a basic graph node for a 2D graph
struct Node
{
  int id;     // this is the id of the node
  double x;   // physical x location
  double y;   // physical y location
  double z;   // physical z location
  double psi;   // yaw orientation of robot
  double theta; // pitch orientation of robot
  double radius; // field used by obstacles, goal, and robot
  double costToNode; // cost of reaching node via current path

  Node* parentNode;
  vector<Node*> neighborNodes;
  Node** robotPts;

  // the following fields are assumed by the heap data structure that I've been using
  //int status;
  bool inHeap;           // init to false and set to true iff the node is in the heap
  int heapIndex;         // enables quick access to the node in the heap

};



// this is the graph data structure
class Graph
{
  public:
    int numObstacles;    // the number of obstacles in the graph
    int numRobotPts;     // the number of points that comprise the robot
    int numNodes;   // the number of nodes in the graph
    float robotRadius; //the radius of the largest extension of the robot with a buffer
    //float X_free; //Lebesgue measure of the obstacles free space

    Node* nodes;     // an array that contains all of the nodes
                     // nodes[i] contains the node with id i

    Node* goal;      // node containing the goal

    Node* obstacles;    // an array that contains all of the obstacles
                        // obstacles[i] contains the obstacle node with id i

    Node* robot;        // an array that contains all of the robot points
                        // robot[i] contains the robot node with id i

    int numEdges;    // the number of edges in the graph

    Edge* edges;     // an array that contains all of the edges
                     // edge[j] contains the j-th edge

    //default constructor
    Graph()
    {
     printf("building a default graph\n");
     numObstacles = 0;
     numRobotPts = 0;
     numNodes = 0;
     //neighborNodes = NULL;
     robotRadius = 0;
     nodes = NULL;
     goal = NULL;
     obstacles = NULL;
     robot = NULL;
     numEdges = 0;
     edges = NULL;
    }

    // default destructor
    ~Graph()
    {
     printf("deleting a graph\n");
     numObstacles = 0;
     numRobotPts = 0;
     numNodes = 0;
     //neighborNodes = NULL;
     robotRadius = 0;
     nodes = NULL;
     goal = NULL;
     obstacles = NULL;
     robot = NULL;
     numEdges = 0;
     edges = NULL;
    }

    bool readObstaclesFromFile(const char* obstacleFilename)
    {
      FILE * pFile;

      // open the node file
      pFile = fopen ( obstacleFilename , "r");
      if(pFile==NULL)
      {
        printf("unable to open file %s\n", obstacleFilename);
        return false;
      }

      // read the number of obstacles
      string line;
      ifstream myfile(obstacleFilename);
      while (getline(myfile, line))
          ++numObstacles;
      //cout << "Number of obstacles is: " << numObstacles<<endl;;


      // allocate space for the nodes
      obstacles = (Node*)calloc(numObstacles, sizeof(Node));

      // now read the nodes one at a time
      float x, y, z, radius;
      for(int n = 0; n < numObstacles; n++)
      {
        if(fscanf(pFile, "%f, %f, %f, %f\n", &x, &y, &z, &radius) < 4)
        {
          printf("problem reading the %i-th node from file\n",n);
          return false;
        }
        // allocate the node
        obstacles[n].id = n;
        obstacles[n].x = x;
        obstacles[n].y = y;
        obstacles[n].z = z;
        obstacles[n].radius = radius;
        obstacles[n].inHeap = false;        // used by heap
        obstacles[n].heapIndex = -1;        // used by heap

        //printf("read node: (%f,%f), %f from file\n", obstacles[n].x, obstacles[n].y, obstacles[n].z, obstacles[n].radius);
      }

      // close the node file
      fclose(pFile);
      return true;
    }

      bool readRobotFromFile(const char* robotFilename)
      {
        FILE * pFile;

        // open the node file
        pFile = fopen ( robotFilename , "r");
        if(pFile==NULL)
        {
          printf("unable to open file %s\n", robotFilename);
          return false;
        }

        // read the number of robot points
        string line;
        ifstream myfile(robotFilename);
        while (getline(myfile, line))
            ++numRobotPts;
        //cout << "Number of Robot Points is: " << numRobotPts<<endl;;

        // allocate space for the nodes
        robot = (Node*)calloc(numRobotPts, sizeof(Node));

        // now read the nodes one at a time
        float x, y, z;
        float tempRadius = 0;
        for(int n = 0; n < numRobotPts; n++)
        {
          if(fscanf(pFile, "%f, %f, %f\n", &x, &y, &z) < 3)
          {
            printf("problem reading the %i-th node from file\n",n);
            return false;
          }
          // allocate the node
          robot[n].id = n;
          robot[n].x = x;
          robot[n].y = y;
          robot[n].z = z;
          robot[n].inHeap = false;        // used by heap
          robot[n].heapIndex = -1;        // used by heap

          tempRadius = sqrt(pow(x,2)+pow(y,2)+pow(z,2));
          if(tempRadius>robotRadius) robotRadius = tempRadius;

          //printf("read node: (%f,%f) from file\n", robot[n].x, robot[n].y, robot[n].z);
        }
      //cout<<"Number of Robot Points at the end of construction: "<< numRobotPts<<endl;
      // close the node file
      fclose(pFile);
      return true;
    }

      // saves the path to a file, extracts it backward from the goal node
      // note that 1 is added to the indicies so that the result is compatible
      // with any 1-indexed input files that may have been used
      // returns true on success
    bool savePathToFile(const char* pathFile, Node* lastNode)
    {
      FILE * pFile = fopen(pathFile,"w");
      if(pFile == NULL)
      {
        return false;
      }
      cout<<"The output path is:"<<endl;
      Node* thisNode = lastNode;
      while(thisNode != NULL)
      {
        if(thisNode->parentNode != NULL){
          cout<<"Node: "<<thisNode->id<<" to Node: "<<thisNode->parentNode->id<<endl;
        }
        // format is id, x, y
        // NOTE: incrimenting ids by 1 to convert to 1-based-indexing
        fprintf(pFile, "%d, %f, %f, %f, %f, %f\n",
                thisNode->id+1, thisNode->x, thisNode->y, thisNode->z,
                thisNode->psi, thisNode->theta);
        thisNode = thisNode->parentNode;
      }

      fclose(pFile);
      printf("saved path in %s\n", pathFile);

      return true;
    }


      // saves the search tree to a file
      // note that 1 is added to the indicies so that the result is compatible
      // with any 1-indexed input files that may have been used
      // returns true on success
    bool saveSearchTreeToFile(const char* searchTreeFile)
    {
      FILE * pFile = fopen(searchTreeFile,"w");
      if(pFile == NULL)
      {
        return false;
      }

      for(int n = 0; n < numNodes; n++)
      {
        Node* thisNode = &nodes[n];
        if(thisNode->parentNode != NULL)
        {

          // format is id1, x1, y1, id2, x2, y2
          // NOTE: incrimenting ids by 1 to convert to 1-based-indexing
          fprintf(pFile, "%d, %f, %f, %f, %d, %f, %f, %f\n",
                  thisNode->id+1, thisNode->x, thisNode->y, thisNode->z,
                  thisNode->parentNode->id+1, thisNode->parentNode->x, thisNode->parentNode->y,thisNode->parentNode->z);
        }
      }
      fclose(pFile);
      printf("saved search tree in %s\n", searchTreeFile);
      printf("number of nodes in search tree: %d\n",numNodes);
      return true;
    }


    // saves the problem criteria to file for Matlab script
    bool saveProblemToFile(const char* problemFile)
    {
      FILE * pFile = fopen(problemFile,"w");
      if(pFile == NULL)
      {
        return false;
      }else{
        fprintf(pFile, "%f, %f, %f\n", nodes[0].x, nodes[0].y, nodes[0].z);
        fprintf(pFile, "%f, %f, %f, %f\n", goal[0].x, goal[0].y, goal[0].z, goal[0].radius);
      }
      fclose(pFile);
      printf("saved problem in %s\n", problemFile);

      return true;
    }


};
