#include <iostream>
#include <vector>
#include <limits>
#include <queue>
#include <cmath>
#include <memory>
#include <unordered_map>
#include <functional>

#include "matplotlibcpp.h"

using namespace std;
namespace plt = matplotlibcpp;

class AStarExample {

public:

  struct sNode {
    bool bObstacle = false;
    bool bVisited = false;
    float fGlobalCost = numeric_limits<float>::max();
    float fLocalCost = numeric_limits<float>::max();
    int x;
    int y;
    std::vector<sNode*> vecNeighbors;
    shared_ptr<sNode> parent = nullptr; 
    
    sNode() {};
    sNode(int x, int y) : x(x), y(y) {};
    sNode(int x, int y, bool bObstacle) : x(x), y(y), bObstacle(bObstacle) {};
  };

  // Array of nodes_
  const int nMapWidth_ = 16;
  const int nMapHeight_ = 16;
  unique_ptr<sNode[]> nodes_;

  shared_ptr<sNode> nodeStart_;
  shared_ptr<sNode> nodeEnd_;
  
  unordered_map<int, shared_ptr<sNode>> mapNodes; 

public:

  AStarExample() {

    nodeStart_ = make_shared<sNode>(1, nMapHeight_ / 2);
    nodeEnd_ = make_shared<sNode>(nMapWidth_ - 2, nMapHeight_ / 2);
    
    mapNodes[GetHash(nodeStart_)] = nodeStart_;
    mapNodes[GetHash(nodeEnd_)] = nodeEnd_;
    
    cout << " constructor " << mapNodes.size() << endl;

    CreateObstacles();
  }

  template<typename T>
  int GetHash(T node) {
    return node->y * nMapWidth_ + node->x;
  }

  void CreateObstacles() {

    int x = nMapWidth_ / 2;
    for (int y = 0; y < nMapHeight_ - 4; y++) {
      auto ptrNode = make_shared<sNode>(x - 3, y, true);
      if (mapNodes.find(GetHash(ptrNode)) == mapNodes.end()) 
        mapNodes[GetHash(ptrNode)] = ptrNode;
    }

    for (int y = 4; y < nMapHeight_; y++) {
      auto ptrNode = make_shared<sNode>(x + 3, y, true);
      if (mapNodes.find(GetHash(ptrNode)) == mapNodes.end()) 
        mapNodes[GetHash(ptrNode)] = ptrNode;
    }

    cout << " plus obstacles " << mapNodes.size() << endl;
    
  }
  
  optional<vector<shared_ptr<sNode>>> SolveAStar() {

    auto calculateDistance = [](auto a, auto b) {
      return std::hypotf(a->x - b->x, a->y - b->y);
    };
    auto calculateL1Distance = [](auto a, auto b) {
      return std::abs(a->x - b->x) + std::abs(a->y - b->y);
    };
    auto calculateHeuristic = [=](auto a, auto b) {
      return calculateDistance(a, b);
    };

    auto nodeCurrent = nodeStart_;
    nodeStart_->fLocalCost = 0.0f;
    nodeStart_->fGlobalCost = calculateHeuristic(nodeStart_, nodeEnd_);
    
    auto cmp = [](const auto a, const auto b) {
      return a->fGlobalCost > b->fGlobalCost;
    };
    
    priority_queue<shared_ptr<sNode>, vector<shared_ptr<sNode>>, decltype(cmp)> 
      queueNotTestedNodes(cmp);
    queueNotTestedNodes.push(nodeStart_);

    int nIterations = 0;

    while (!queueNotTestedNodes.empty()) {
      nIterations++;

      // If we have already visited the nodes_, pop them
      while (!queueNotTestedNodes.empty() && queueNotTestedNodes.top()->bVisited)
        queueNotTestedNodes.pop();

      if (queueNotTestedNodes.empty()) {
        break; }
      
      auto nodeCurrent = queueNotTestedNodes.top(); 
      nodeCurrent->bVisited = true;

      auto vecNodeNeighbors = GenerateNeighbors(nodeCurrent);  
      
      for (auto nodeNeighbor : vecNodeNeighbors) {

        float fPossiblyLowerCost = nodeCurrent->fLocalCost 
          + calculateDistance(nodeCurrent, nodeNeighbor);

        if (fPossiblyLowerCost < nodeNeighbor->fLocalCost
            && fPossiblyLowerCost < nodeEnd_->fLocalCost) {

          nodeNeighbor->parent = nodeCurrent;
          nodeNeighbor->fLocalCost = fPossiblyLowerCost; 
          nodeNeighbor->fGlobalCost = nodeNeighbor->fLocalCost 
            + calculateHeuristic(nodeNeighbor, nodeEnd_);

          if (!nodeNeighbor->bObstacle && nodeNeighbor->fGlobalCost <= nodeEnd_->fGlobalCost) {
            queueNotTestedNodes.push(nodeNeighbor);
          }
        }
      }
    }

    cout << " iterations: " << nIterations << endl;

    if (nodeEnd_->bVisited) {
      cout << " Solution found!! ";
      return GetPathToEnd();
    } else {
      cout << " Solution not found :( ";
      return {};
    }

  }
  
  vector<shared_ptr<sNode>> GenerateNeighbors(shared_ptr<sNode>& node) {
    
    vector<std::shared_ptr<sNode>> candidates;
    
    if (node->x > 0)
      candidates.push_back(make_shared<sNode>(node->x - 1, node->y + 0));
    if (node->x < nMapWidth_ - 1)
      candidates.push_back(make_shared<sNode>(node->x + 1, node->y + 0));
    if (node->y > 0)
      candidates.push_back(make_shared<sNode>(node->x + 0, node->y - 1));
    if (node->y < nMapWidth_ - 1)
      candidates.push_back(make_shared<sNode>(node->x + 0, node->y + 1));

    if (node->y > 0 && node->x > 0) 
      candidates.push_back(make_shared<sNode>(node->x - 1, node->y - 1));
    if (node->y < nMapHeight_ - 1 && node->x < nMapWidth_ - 1) 
      candidates.push_back(make_shared<sNode>(node->x + 1, node->y + 1));
    if (node->y < nMapHeight_ - 1 && node->x > 0) 
      candidates.push_back(make_shared<sNode>(node->x - 1, node->y + 1));
    if (node->y > 0 && node->x < nMapWidth_ - 1) 
      candidates.push_back(make_shared<sNode>(node->x + 1, node->y - 1));
    
    vector<shared_ptr<sNode>> vecNeighbors;
    for (auto nodeNeighbor : candidates) {
      // If it is not there add it
      if (mapNodes.find(GetHash(nodeNeighbor)) == mapNodes.end()) {
        mapNodes[GetHash(nodeNeighbor)] = nodeNeighbor;
      }
           
      vecNeighbors.push_back(mapNodes[GetHash(nodeNeighbor)]);
    }
    
    return vecNeighbors;
  }

  int CalculateVisitedNodes() {
    return mapNodes.size();
  }
  
  vector<shared_ptr<sNode>> GetPathToEnd() {
    auto p = nodeEnd_;
    vector<shared_ptr<sNode>> pathToEnd; 

    while (p->parent != nullptr) {
      pathToEnd.push_back(p);
      p = p->parent;
    }

    pathToEnd.push_back(p);
    return pathToEnd;
  }

  /////////////////////////// Plotting Utilities //////////////////////////

  void Visualize(auto optPath) {

    if (optPath) {
        auto vecPathToEnd = optPath.value();
        vector<float> vecX, vecY;
    
        for (auto node : vecPathToEnd) {
          cout << "x: " << node->x << " y: " << node->y;
          cout << " fGlobalCost: " << node->fGlobalCost << endl;
          vecX.push_back(node->x);
          vecY.push_back(node->y);
        }
    
        plt::plot(vecX,vecY, {{"color", "r"}, {"linewidth", "3"}});
    }

    for (const auto &[key, node] : mapNodes) {
        if (node->bObstacle)
          PlotBox(node->x, node->y, {{"color", "r"}});
        else if (node->bVisited)
          PlotBox(node->x, node->y, {{"color", "g"}});
        else
          PlotBox(node->x, node->y, {{"color", "b"}});
    }

    PlotBox(nodeStart_->x, nodeStart_->y, {{"color", "k"}});
    PlotBox(nodeEnd_->x, nodeEnd_->y, {{"color", "k"}});

  }
  
  void PlotBox(float x, float y, const map<string, string>& keywords) {
    float w = 0.1;
    float h = 0.1;
    vector<float> xcoor = {x - w, x + w, x + w, x - w, x - w};
    vector<float> ycoor = {y - h, y - h, y + h, y + h, y - h};
    plt::fill(xcoor, ycoor, keywords);
  }


};

int main(int argc, char *argv[]) {
  AStarExample aStarExample = AStarExample(); 
  auto optPath = aStarExample.SolveAStar();

  // Setup the plot
  plt::figure_size(1200, 780);
  aStarExample.Visualize(optPath);
  plt::title("astar");
  plt::show();

  cout << "visitedNodes: " << aStarExample.CalculateVisitedNodes() << endl;

  return 0;
}

