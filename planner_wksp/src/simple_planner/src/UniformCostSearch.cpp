//
// Created by Salvatore  Rago on 15/02/24.
//

#include "UniformCostSearch.h"

UniformCostSearch::UniformCostSearch(ImageMap map) {
    this->map = map;
}

Eigen::MatrixXi UniformCostSearch::computeRowDerivative(Eigen::MatrixXi map) const {
    int num_rows = map.rows();
    int num_cols = map.cols();

    Eigen::MatrixXi res = Eigen::MatrixXi::Zero(num_rows, num_cols);

    for (int i = 1; i < num_rows - 1; i++) {
        for (int j = 1; j < num_cols - 1; j++) {
            res(i, j) = map(i + 1, j) - map(i - 1, j);
        }
    }

    return res;
}

Eigen::MatrixXi UniformCostSearch::computeColumnDerivative(Eigen::MatrixXi map) const {
    int num_rows = map.rows();
    int num_cols = map.cols();

    Eigen::MatrixXi res = Eigen::MatrixXi::Zero(num_rows, num_cols);

    for (int i = 1; i < num_rows - 1; i++) {
        for (int j = 1; j < num_cols - 1; j++) {
            res(i, j) = map(i, j + 1) - map(i, j - 1);
        }
    }

    return res;
}

Eigen::MatrixXi UniformCostSearch::computeMagnitudeDerivative(Eigen::MatrixXi map) const {
    int num_rows = map.rows();
    int num_cols = map.cols();

    Eigen::MatrixXi res = Eigen::MatrixXi::Zero(num_rows, num_cols);

    // Compute the row and column derivatives on the given map
    Eigen::MatrixXi rowDerivative = computeRowDerivative(map);
    Eigen::MatrixXi colDerivative = computeColumnDerivative(map);

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
 * Computes the distance of each cell of the map (given as matrix) from the closest obstacle.
 * @param map The matrix that represents the map. The obstacles are cells containing OBSTACLE,
 * while free cells contain FREE_CELL.
 * @return The computed distance map.
 */
Eigen::MatrixXi UniformCostSearch::computeDistanceMap(Eigen::MatrixXi map) const {
    int num_rows = map.rows();
    int num_cols = map.cols();

    Eigen::MatrixXi res;
    res.resize(num_rows, num_cols);

    for (int i = 0; i < num_rows; i++) {
        for (int j = 0; j < num_cols; j++) {
            // The current cell is an obstacle, then set the distance to 0
            if (map(i, j) == OBSTACLE) {
                res(i, j) = 0;
            } else {
                // Look for the closest obstacle of the current cell in a neighborhood with a progressively bigger radius
                int dist = 1;
                bool inBounds = i - dist >= 0 || i + dist < num_rows || j - dist >= 0 || j + dist < num_cols;
                bool obstacleFound = false;
                while (inBounds && !obstacleFound) {
                    int minBoundRow = std::max(i - dist, 0);
                    int maxBoundRow = std::min(num_rows, i + dist + 1);
                    int minBoundCol = std::max(j - dist, 0);
                    int maxBoundCol = std::min(num_cols, j + dist + 1);
                    for (int u = minBoundRow; u < maxBoundRow && !obstacleFound; u++) {
                        for (int v = minBoundCol; v < maxBoundCol && !obstacleFound; v++) {
                            // The obstacle is found, then stop searching
                            if (map(u, v) == OBSTACLE) {
                                res(i, j) = dist;
                                obstacleFound = true;
                            }
                        }
                    }

                    dist++;
                    inBounds = i - dist >= 0 || i + dist < num_rows || j - dist >= 0 || j + dist < num_cols;
                }
            }
        }
    }

    return res;
}