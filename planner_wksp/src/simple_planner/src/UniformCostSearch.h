//
// Created by Salvatore  Rago on 15/02/24.
//

#ifndef SIMPLE_PLANNER_UNIFORMCOSTSEARCH_H
#define SIMPLE_PLANNER_UNIFORMCOSTSEARCH_H

#include "ImageMap.h"
#include "Eigen/Dense"


class UniformCostSearch {
public:
    UniformCostSearch(ImageMap map);

    static const int OBSTACLE = 100;
    static const int FREE_CELL = 0;

    ImageMap map;
    Eigen::MatrixXi computeMagnitudeDerivative(Eigen::MatrixXi map) const;
    Eigen::MatrixXi computeRowDerivative(Eigen::MatrixXi map) const;
    Eigen::MatrixXi computeColumnDerivative(Eigen::MatrixXi map) const;
    Eigen::MatrixXi computeDistanceMap(Eigen::MatrixXi map) const;
// private:
};


#endif //SIMPLE_PLANNER_UNIFORMCOSTSEARCH_H
