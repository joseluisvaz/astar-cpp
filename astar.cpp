#include <iostream>
#include <vector>
#include <limits>
#include <queue>
#include <cmath>
#include <memory>
#include <list>

#include "matplotlibcpp.h"

using namespace std;
namespace plt = matplotlibcpp;

class AStarExample {

public:

  struct sNode {
    bool bObstacle;
    bool bVisited;
    float fGlobalCost;
    float fLocalCost;
    int x;
    int y;
    std::vector<sNode*> vecNeighbors;
    sNode* parent; 
  };

  // Array of nodes_
  const int nMapWidth_ = 16;
  const int nMapHeight_ = 16;
  unique_ptr<sNode[]> nodes_;

  sNode *nodeStart_ = nullptr;
  sNode *nodeEnd_ = nullptr;

public:

  AStarExample() {
    // Start a grid
    nodes_ = std::make_unique<sNode[]>(nMapWidth_ * nMapHeight_);
    for (int x = 0; x < nMapWidth_; x++) {
      for (int y = 0; y < nMapHeight_; y++) {
        nodes_[y * nMapWidth_ + x].x = x;
        nodes_[y * nMapWidth_ + x].y = y;
        nodes_[y * nMapWidth_ + x].bObstacle = false;
        nodes_[y * nMapWidth_ + x].bVisited = false;
        nodes_[y * nMapWidth_ + x].parent = nullptr;
      }
    }
    
    // Create connection between nodes_
    for (int x = 0; x < nMapWidth_; x++) {
      for (int y = 0; y < nMapHeight_; y++) {
        if (y > 0) 
          nodes_[y * nMapWidth_ + x].vecNeighbors.push_back(&nodes_[(y - 1) * nMapWidth_ + (x + 0)]);
        if (y < nMapHeight_ - 1) 
          nodes_[y * nMapWidth_ + x].vecNeighbors.push_back(&nodes_[(y + 1) * nMapWidth_ + (x + 0)]);
        if (x > 0) 
          nodes_[y * nMapWidth_ + x].vecNeighbors.push_back(&nodes_[(y + 0) * nMapWidth_ + (x - 1)]);
        if (x < nMapWidth_ - 1) 
          nodes_[y * nMapWidth_ + x].vecNeighbors.push_back(&nodes_[(y + 0) * nMapWidth_ + (x + 1)]);

        // Conncet diagonally
        if (y > 0 && x > 0) 
          nodes_[y * nMapWidth_ + x].vecNeighbors.push_back(&nodes_[(y - 1) * nMapWidth_ + (x - 1)]);
        if (y < nMapHeight_ - 1 && x < nMapWidth_ - 1) 
          nodes_[y * nMapWidth_ + x].vecNeighbors.push_back(&nodes_[(y + 1) * nMapWidth_ + (x + 1)]);
        if (y < nMapHeight_ - 1 && x > 0) 
          nodes_[y * nMapWidth_ + x].vecNeighbors.push_back(&nodes_[(y + 1) * nMapWidth_ + (x - 1)]);
        if (y > 0 && x < nMapWidth_ - 1) 
          nodes_[y * nMapWidth_ + x].vecNeighbors.push_back(&nodes_[(y - 1) * nMapWidth_ + (x + 1)]);
      }
    }

    nodeStart_ = &nodes_[(nMapHeight_ / 2) * nMapWidth_ + 1];
    nodeEnd_ = &nodes_[(nMapHeight_ / 2) * nMapWidth_ + nMapWidth_-2];

    CreateObstacles();

  }

  void CreateObstacles() {

    int x = nMapWidth_ / 2;
    for (int y = 0; y < nMapHeight_ - 5; y++) {
      nodes_[y * nMapWidth_ + x - 4].bObstacle = true;
    }

    for (int y = 4; y < nMapHeight_; y++) {
      nodes_[y * nMapWidth_ + x].bObstacle = true;
    }
    
  }

  
  void SolveAStar() {

    // Reset nodes_
    for (int x = 0; x < nMapWidth_; x++) {
      for (int y = 0; y < nMapHeight_; y++) {
        nodes_[y * nMapWidth_ + x].bVisited = false;
        nodes_[y * nMapWidth_ + x].parent = nullptr;
        nodes_[y * nMapWidth_ + x].fGlobalCost = numeric_limits<float>::infinity();
        nodes_[y * nMapWidth_ + x].fLocalCost = numeric_limits<float>::infinity();
      }
    }

    auto calculateDistance = [](sNode *a, sNode *b) {
      return std::hypotf(a->x - b->x, a->y - b->y);
    };
    auto calculateL1Distance = [](sNode *a, sNode *b) {
      return std::abs(a->x - b->x) + std::abs(a->y - b->y);
    };
    
    auto calculateHeuristic = [=](sNode *a, sNode *b) {
      return calculateL1Distance(a, b);
    };

    sNode *nodeCurrent = nodeStart_;
    nodeStart_->fLocalCost = 0.0f;
    nodeStart_->fGlobalCost = calculateHeuristic(nodeStart_, nodeEnd_);
    
    auto cmp = [](const sNode *a, const sNode *b) {
      return a->fGlobalCost > b->fGlobalCost;
    };
    
    priority_queue<sNode*, vector<sNode*>, decltype(cmp)> queueNotTestedNodes(cmp);
    queueNotTestedNodes.push(nodeStart_);

    int nIterations = 0;

    while (!queueNotTestedNodes.empty()) {
      nIterations++;

      // If we have already visited the nodes_, pop them
      while (!queueNotTestedNodes.empty() && queueNotTestedNodes.top()->bVisited)
        queueNotTestedNodes.pop();

      if (queueNotTestedNodes.empty())
        break;
      
      auto nodeCurrent = queueNotTestedNodes.top(); 
      nodeCurrent->bVisited = true;

      // Update and add neighbors if not visited
      for (auto nodeNeighbor : nodeCurrent->vecNeighbors) {

        float fPossiblyLowerCost = nodeCurrent->fLocalCost 
          + calculateDistance(nodeCurrent, nodeNeighbor);

        if (fPossiblyLowerCost < nodeNeighbor->fLocalCost
            && fPossiblyLowerCost < nodeEnd_->fLocalCost) {

          nodeNeighbor->parent = nodeCurrent;
          nodeNeighbor->fLocalCost = fPossiblyLowerCost; 
          nodeNeighbor->fGlobalCost = nodeNeighbor->fLocalCost 
            + calculateHeuristic(nodeNeighbor, nodeEnd_);

          if (!nodeNeighbor->bObstacle && nodeNeighbor->fGlobalCost < nodeEnd_->fGlobalCost) {
            queueNotTestedNodes.push(nodeNeighbor);
          }
        }
      }
    }

    if (nodeEnd_->bVisited) {
      cout << " Solution found!! ";
    } else {
      cout << " Solution not found :( ";
    }

    cout << " iterations: " << nIterations << endl;
  }

  int CalculateVisitedNodes() {
    int nVisitedNodes = 0; 
    for (int x = 0; x < nMapWidth_; x++) {
      for (int y = 0; y < nMapHeight_; y++) {
        if (nodes_[y * nMapWidth_ + x].bVisited) {
           nVisitedNodes++;
        }
      }
    }
    return nVisitedNodes;
  }
  
  vector<sNode*> GetPathToEnd() {
    sNode *p = nodeEnd_;
    vector<sNode*> pathToEnd; 

    while (p->parent != nullptr) {
      pathToEnd.push_back(p);
      p = p->parent;
    }

    pathToEnd.push_back(p);
    return pathToEnd;
  }

  /////////////////////////// Plotting Utilities //////////////////////////

  void plotNodes() {

    for (int i = 0; i < nMapWidth_ * nMapHeight_; i++) {
        if (nodes_[i].bObstacle)
          plotBox(nodes_[i].x, nodes_[i].y, "r");
        else if (nodes_[i].bVisited)
          plotBox(nodes_[i].x, nodes_[i].y, "g");
        else
          plotBox(nodes_[i].x, nodes_[i].y, "b");
    }

    plotBox(nodeStart_->x, nodeStart_->y, "g");
    plotBox(nodeEnd_->x, nodeEnd_->y, "r");

  }
  
  void plotConnections(sNode *node) {
    for (auto neighbor : node->vecNeighbors) {
      vector<int> xcoor = {node->x, neighbor->x};
      vector<int> ycoor = {node->y, neighbor->y};
      plt::plot(xcoor, ycoor, {{"color", "y"}});
    }
  }
  
  void plotBox(float x, float y, string &&s) {
    float w = 0.2;
    float h = 0.2;
    vector<float> xcoor = {x - w, x + w, x + w, x - w, x - w};
    vector<float> ycoor = {y - h, y - h, y + h, y + h, y - h};
    plt::fill(xcoor, ycoor, {{"color", s}});
  }


};

int main(int argc, char *argv[]) {
  AStarExample aStarExample = AStarExample(); 
  aStarExample.SolveAStar();

  vector<AStarExample::sNode*> vecPathToEnd = aStarExample.GetPathToEnd(); 

  for (auto node : vecPathToEnd) {
    cout << "x: " << node->x << " y: " << node->y;
    cout << " fGlobalCost: " << node->fGlobalCost << endl;
  }

  cout << "visitedNodes: " << aStarExample.CalculateVisitedNodes() << endl;
  
  vector<float> vecX, vecY;

  for (auto node : vecPathToEnd) {
    vecX.push_back(node->x);
    vecY.push_back(node->y);
  }
  
  // Setup the plot
  
  plt::figure_size(1200, 780);
  aStarExample.plotNodes();
  plt::plot(vecX,vecY, {{"color", "r"}, {"linewidth", "3"}});

  plt::title("astar");
  plt::show();

  return 0;
}

