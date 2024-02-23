//
// Created by Salvatore  Rago on 15/02/24.
//

#ifndef SIMPLE_PLANNER_UNIFORMCOSTSEARCH_H
#define SIMPLE_PLANNER_UNIFORMCOSTSEARCH_H

#include "ImageMap.h"
#include "Eigen/Dense"
#include "queue"

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

class UniformCostSearch {
public:
    UniformCostSearch(ImageMap map);

    static const int OBSTACLE = 100;
    static const int FREE_CELL = 0;

    ImageMap map;

    Eigen::MatrixXf computeMagnitudeDerivative(Eigen::MatrixXf map) const;

    Eigen::MatrixXf computeRowDerivative(Eigen::MatrixXf map) const;

    Eigen::MatrixXf computeColumnDerivative(Eigen::MatrixXf map) const;

    // Eigen::MatrixXf computeDistanceMap(Eigen::MatrixXi map) const;
    Eigen::MatrixXf computeDistanceMap(Eigen::MatrixXi map, float maximumDistance);
// private:
};


#endif //SIMPLE_PLANNER_UNIFORMCOSTSEARCH_H
