//
// Created by Salvatore  Rago on 15/02/24.
//

#ifndef SIMPLE_PLANNER_PathFinding_H
#define SIMPLE_PLANNER_PathFinding_H

#include "ImageMap.h"
#include "Eigen/Dense"
#include "queue"
#include <list>
#include <set>
#include "limits.h"

// The cell used to compute the distance map
struct DistanceMapCell {
    // The parent of this cell
    DistanceMapCell *parent = nullptr;
    // The distance of this cell from its parent
    float distanceFromParent = -1;
    // The coordinates of this cell
    int x = -1;
    int y = -1;
};

// The node used during the uniform gCost search
class SearchNode {
public:
    int x;
    int y;
    float gCost;
    float hCost;
    // The node used to reach this node (nullptr if it is the initial node)
    SearchNode *parentNode;

    SearchNode(int x, int y, float gCost, float hCost, SearchNode *parentNode = nullptr) :
            x(x), y(y),
            gCost(gCost), hCost(hCost),
            parentNode(parentNode) {};

    float getOverallCost() const { return gCost + hCost; }

    static float getHCostFromVoronoiMap(const Eigen::MatrixXf *voronoiMap, int x, int y) {
        return (*voronoiMap)(x, y);
    }
};

class SearchNodesComparator {
public:
    int operator()(const SearchNode &n1, const SearchNode &n2) {
        return n1.getOverallCost() < n2.getOverallCost();
    }
};

class PathFinding {
public:
    PathFinding(ImageMap map);

    static const int OBSTACLE = 100;
    static const int FREE_CELL = 0;

    ImageMap map;

    std::list <Eigen::Vector2i>
    performPathFinding(const Eigen::Vector2i &initialPosition, const Eigen::Vector2i &goalPosition);

private:
    Eigen::MatrixXf voronoiMap;

    std::list <Eigen::Vector2i> getPathFromNode(SearchNode *node);

    SearchNode *addNodeToClosedList(std::list <SearchNode> *closedList, SearchNode &toAdd);

    std::multiset<SearchNode, SearchNodesComparator>::iterator
    findElementByCoordsInSet(int x, int y, std::multiset <SearchNode, SearchNodesComparator> *set);

    Eigen::MatrixXf computeMagnitudeDerivative(Eigen::MatrixXf distanceMap) const;

    Eigen::MatrixXf computeRowDerivative(Eigen::MatrixXf map) const;

    Eigen::MatrixXf computeColumnDerivative(Eigen::MatrixXf map) const;

    Eigen::MatrixXf computeDistanceMap(Eigen::MatrixXi map, float maximumDistance);
};


#endif //SIMPLE_PLANNER_PathFinding_H
