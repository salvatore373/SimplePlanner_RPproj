//
// Created by Salvatore  Rago on 15/02/24.
//

#include "PathFinding.h"

PathFinding::PathFinding(ImageMap map) {
    this->map = map;

    // Compute the voronoi diagram of the map (from the distance map), and multiply by a constant to
    // make the voronoi values more relevant in Astar.
    Eigen::MatrixXf distanceMap = computeDistanceMap(map.grid, 50);
    voronoiMap = computeMagnitudeDerivative(distanceMap) * 100;
}

Eigen::MatrixXf PathFinding::computeRowDerivative(Eigen::MatrixXf map) const {
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

Eigen::MatrixXf PathFinding::computeColumnDerivative(Eigen::MatrixXf map) const {
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

Eigen::MatrixXf PathFinding::computeMagnitudeDerivative(Eigen::MatrixXf distanceMap) const {
    int num_rows = distanceMap.rows();
    int num_cols = distanceMap.cols();

    Eigen::MatrixXf res = Eigen::MatrixXf::Zero(num_rows, num_cols);

    // Compute the row and column derivatives on the given distanceMap
    Eigen::MatrixXf rowDerivative = computeRowDerivative(distanceMap);
    Eigen::MatrixXf colDerivative = computeColumnDerivative(distanceMap);

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
Eigen::MatrixXf PathFinding::computeDistanceMap(Eigen::MatrixXi map, float maximumDistance) {
    int num_rows = map.rows();
    int num_cols = map.cols();

    // Add all the obstacles to the frontier queue and set the parent of the obstacle cells
    Eigen::Matrix <DistanceMapCell, Eigen::Dynamic, Eigen::Dynamic> distanceCells(num_rows, num_cols);
    std::queue < DistanceMapCell * > frontier;
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

                if (dist > maximumDistance) {
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


std::list <Eigen::Vector2i> PathFinding::getPathFromNode(SearchNode *node) {
    // Create the vector that will contain the path points
    std::list <Eigen::Vector2i> path;

    SearchNode *currNode = node;
    while (currNode != nullptr) {
        path.emplace_front(currNode->x, currNode->y);
        currNode = currNode->parentNode;
    }

    return path;
}

SearchNode *PathFinding::addNodeToClosedList(std::list <SearchNode> *closedList, SearchNode &toAdd) {
    for (auto itr = closedList->begin(); itr != closedList->end(); itr++) {
        SearchNode n = *itr;
        if (n.x == toAdd.x && n.y == toAdd.y) {
            // Only change the parent node and the cost value if this is the new version of the same node already
            // in the closed list
            itr->gCost = toAdd.gCost;
            itr->hCost = toAdd.hCost;
            itr->parentNode = toAdd.parentNode;
            return &(*itr);
        }
    }
    closedList->push_back(toAdd);
    return &closedList->back();


}

std::multiset<SearchNode, SearchNodesComparator>::iterator
PathFinding::findElementByCoordsInSet(int x, int y, std::multiset <SearchNode, SearchNodesComparator> *set) {
    for (auto itr = set->begin(); itr != set->end(); itr++) {
        SearchNode n = *itr;
        if (n.x == x && n.y == y) return itr;
    }

    // No element with the given coordinates was found in the given set
    return set->end();
}

/**
 * Returns the closest element in closedList to the given goalPosition.
 */
SearchNode *getClosestNodeToGoal(std::list <SearchNode> *closedList, const Eigen::Vector2i &goalPosition) {
    closedList->sort([goalPosition](const SearchNode &n1, const SearchNode &n2) {
        return (goalPosition - Eigen::Vector2i(n1.x, n1.y)).norm() <
               (goalPosition - Eigen::Vector2i(n2.x, n2.y)).norm();
    });

    return &(closedList->front());
}

/**
 * Computes the best path from the initial position to the goal, according to the A* algorithm.
 * In the following implementation of the A* algorithm the values of the cells of the Voronoi diagram are used as
 * h-cost, while the g-cost is the number of steps from the initial position to the current node.
 * @param initialPosition The initial cell in the map.
 * @param goalPosition The goal cell in the map.
 * @return The path from the initial to the goal position, which consist in a vector of cells positions.
 * If the goal is not reachable from the provided initial position it returns the path to the closest position
 * to the goal.
 */
std::list <Eigen::Vector2i>
PathFinding::performPathFinding(const Eigen::Vector2i &initialPosition,
                                const Eigen::Vector2i &goalPosition) {
    std::cout << "Starting to compute the best path from the initial to the goal position..." << std::endl;

    int num_rows = voronoiMap.rows();
    int num_cols = voronoiMap.cols();

    // Create the vector that will contain the explored
    std::list <SearchNode> closedList;
    // Create the map of visited positions
    Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic> visited;
    visited = Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic>::Constant(num_rows, num_cols, false);
    // Create a min priority queue (min heap)
    // std::priority_queue<SearchNode, std::vector<SearchNode>, SearchNodesComparator> pQueue;
    std::multiset <SearchNode, SearchNodesComparator> pQueue;

    // Add initial position to priority queue (with gCost 0)
    pQueue.emplace(initialPosition.x(), initialPosition.y(),
                   0, SearchNode::getHCostFromVoronoiMap(&voronoiMap, initialPosition.x(), initialPosition.y()));

    SearchNode *nodePtr;
    while (!pQueue.empty()) {
        // Get the top of the queue (the node with minimum gCost)
        SearchNode node = *pQueue.begin();
        pQueue.erase(pQueue.begin());
        SearchNode *nodePtr = addNodeToClosedList(&closedList, node);
        Eigen::Vector2i currPos(node.x, node.y);

        // Mark this node as visited
        visited(node.x, node.y) = true;

        // Check whether the current node is the goal
        if (node.x == goalPosition.x() && node.y == goalPosition.y()) {
            std::cout << "The path has been found" << std::endl;
            return getPathFromNode(nodePtr);
        }

        int minBoundRow = std::max(currPos.x() - 1, 0);
        int maxBoundRow = std::min(num_rows, currPos.x() + 1 + 1);
        int minBoundCol = std::max(currPos.y() - 1, 0);
        int maxBoundCol = std::min(num_cols, currPos.y() + 1 + 1);
        for (int u = minBoundRow; u < maxBoundRow; ++u) {
            for (int v = minBoundCol; v < maxBoundCol; ++v) {
                if (u == currPos.x() && v == currPos.y()) continue;
                // Do not allow movements along the diagonal
                if (u != currPos.x() && v != currPos.y()) continue;
                // Do not allow to include an obstacle in the path
                if (map.grid(u, v) == OBSTACLE) continue;

                float neighborGCost = node.gCost + 1;
                float neighborHCost = SearchNode::getHCostFromVoronoiMap(&voronoiMap, u, v);
                auto oldNode = findElementByCoordsInSet(u, v, &pQueue);
                if (!visited(u, v) && oldNode == pQueue.end()) {
                    // This neighbor has not been visited nor added to the queue yet, then add it to the queue
                    pQueue.emplace(u, v,
                                   neighborGCost, neighborHCost,
                                   nodePtr);
                } else if (oldNode != pQueue.end() && oldNode->getOverallCost() > neighborGCost + neighborHCost) {
                    // This node has already benn explored, but we found a shorter path, then we make a replacement
                    pQueue.erase(oldNode);
                    pQueue.emplace(u, v,
                                   neighborGCost, neighborHCost,
                                   nodePtr);
                }
            }
        }
    }

    // Unable to find the path to the goal
    std::cerr << "Unable to find a path from the initial provided position to the goal." << std::endl;
    return getPathFromNode(getClosestNodeToGoal(&closedList, goalPosition));
}
