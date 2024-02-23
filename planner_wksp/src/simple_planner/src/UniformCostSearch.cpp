//
// Created by Salvatore  Rago on 15/02/24.
//

#include "UniformCostSearch.h"

UniformCostSearch::UniformCostSearch(ImageMap map) {
    this->map = map;
}

Eigen::MatrixXf UniformCostSearch::computeRowDerivative(Eigen::MatrixXf map) const {
    int num_rows = map.rows();
    int num_cols = map.cols();

    Eigen::MatrixXf res = Eigen::MatrixXf::Zero(num_rows, num_cols);

    for (int i = 1; i < num_rows - 1; i++) {
        for (int j = 1; j < num_cols - 1; j++) {
            res(i, j) = (float) (map(i + 1, j) - map(i - 1, j));
        }
    }

    return res;
}

Eigen::MatrixXf UniformCostSearch::computeColumnDerivative(Eigen::MatrixXf map) const {
    int num_rows = map.rows();
    int num_cols = map.cols();

    Eigen::MatrixXf res = Eigen::MatrixXf::Zero(num_rows, num_cols);

    for (int i = 1; i < num_rows - 1; i++) {
        for (int j = 1; j < num_cols - 1; j++) {
            res(i, j) = (float) (map(i, j + 1) - map(i, j - 1));
        }
    }

    return res;
}

Eigen::MatrixXf UniformCostSearch::computeMagnitudeDerivative(Eigen::MatrixXf map) const {
    int num_rows = map.rows();
    int num_cols = map.cols();

    Eigen::MatrixXf res = Eigen::MatrixXf::Zero(num_rows, num_cols);

    // Compute the row and column derivatives on the given map
    Eigen::MatrixXf rowDerivative = computeRowDerivative(map);
    Eigen::MatrixXf colDerivative = computeColumnDerivative(map);

    for (int r = 0; r < num_rows; ++r) {
        for (int c = 0; c < num_cols; ++c) {
            // Put in a vector the derivatives for the current cell and apply squared normalization
            Eigen::Vector2f cell_derivatives(rowDerivative(r, c), colDerivative(r, c));
            res(r, c) = cell_derivatives.squaredNorm();
        }
    }

    return res;
}

/**
 * Compute the distance of each cell of map (given as matrix) from the closest obstacle. The cells that are at a
 * distance greater than maximumDistance from the closest obstacle will be considered all at distance maximumDistance.
 * @param map The matrix that represents the map. The obstacles are cells containing OBSTACLE,
 * while free cells contain FREE_CELL.
 * @return The matrix that associates each cell to the distance from the closest obstacle.
 */
Eigen::MatrixXf UniformCostSearch::computeDistanceMap(Eigen::MatrixXi map, float maximumDistance) {
    int num_rows = map.rows();
    int num_cols = map.cols();

    // Add all the obstacles to the frontier queue and set the parent of the obstacle cells
    Eigen::Matrix<DistanceMapCell, Eigen::Dynamic, Eigen::Dynamic> distanceCells(num_rows, num_cols);
    std::queue<DistanceMapCell *> frontier;
    for (int i = 0; i < num_rows; ++i) {
        for (int j = 0; j < num_cols; ++j) {
            DistanceMapCell *dCell = &distanceCells(i, j);
            dCell->x = i;
            dCell->y = j;

            if (map(i, j) == OBSTACLE) {
                dCell->parent = dCell;
                dCell->distanceFromParent = 0;
                frontier.push(dCell);
            } else {
                dCell->distanceFromParent = maximumDistance;
            }
        }
    }

    // Set the parent of the neighborhood of the cells in the queue
    while (!frontier.empty()) {
        // Take the head of the queue
        DistanceMapCell *elem = frontier.front();
        frontier.pop();

        // Take the coordinates of the parent
        Eigen::Vector2f elemParent(elem->parent->x, elem->parent->y);

        // Explore the neighborhood of the element
        int minBoundRow = std::max(elem->x - 1, 0);
        int maxBoundRow = std::min(num_rows, elem->x + 1 + 1);
        int minBoundCol = std::max(elem->y - 1, 0);
        int maxBoundCol = std::min(num_cols, elem->y + 1 + 1);
        for (int u = minBoundRow; u < maxBoundRow; u++) {
            for (int v = minBoundCol; v < maxBoundCol; v++) {
                if (u == elem->x && v == elem->y) continue;

                DistanceMapCell *neighbor = &distanceCells(u, v);
                Eigen::Vector2f neighborCoords(u, v);
                float dist = (neighborCoords - elemParent).norm();

                if(dist > maximumDistance) {
                    continue;
                }

                if (neighbor->parent) {
                    Eigen::Vector2f neighborParentCoords(neighbor->parent->x, neighbor->parent->y);

                    if (dist >= (neighborCoords - neighborParentCoords).norm()) {
                        // The parent of the neighbor don't have to be updated
                        continue;
                    }
                }

                // Update the neighbor parent and check the parent of its neighborhood
                neighbor->parent = elem->parent;
                neighbor->distanceFromParent = dist;
                neighbor->x = u;
                neighbor->y = v;
                frontier.push(neighbor);
            }
        }
    }

    // Convert the distance map to a matrix of distances
    Eigen::MatrixXf res;
    res.resize(num_rows, num_cols);
    for (int i = 0; i < num_rows; ++i) {
        for (int j = 0; j < num_cols; ++j) {
            res(i, j) = distanceCells(i, j).distanceFromParent;
        }
    }
    return res;
}