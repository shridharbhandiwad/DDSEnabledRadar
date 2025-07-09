#pragma once

#include "common/types.hpp"
#include <vector>
#include <utility>

namespace radar {
namespace processing {
namespace association {

/**
 * @brief Hungarian Algorithm implementation for solving assignment problems
 * 
 * This class implements the Hungarian algorithm (also known as the Munkres algorithm)
 * for solving the assignment problem in polynomial time O(n^3).
 */
class HungarianAlgorithm {
public:
    /**
     * @brief Structure to hold assignment result
     */
    struct AssignmentResult {
        std::vector<std::pair<int, int>> assignment; ///< Pairs of (row, column) assignments
        double total_cost{0.0};                      ///< Total cost of the assignment
        bool is_valid{false};                        ///< Whether the assignment is valid
    };

    /**
     * @brief Solve the assignment problem (minimization)
     * @param cost_matrix The cost matrix where cost_matrix(i,j) is the cost of assigning row i to column j
     * @return AssignmentResult containing the optimal assignment
     */
    static AssignmentResult solve(const common::MatrixXd& cost_matrix);

    /**
     * @brief Solve the assignment problem for maximization
     * @param profit_matrix The profit matrix where profit_matrix(i,j) is the profit of assigning row i to column j
     * @return AssignmentResult containing the optimal assignment
     */
    static AssignmentResult solveMaximization(const common::MatrixXd& profit_matrix);

    /**
     * @brief Calculate the total cost of a given assignment
     * @param cost_matrix The cost matrix
     * @param assignment Vector of (row, column) pairs representing the assignment
     * @return Total cost of the assignment
     */
    static double calculateAssignmentCost(
        const common::MatrixXd& cost_matrix,
        const std::vector<std::pair<int, int>>& assignment);

    /**
     * @brief Validate an assignment
     * @param assignment Vector of (row, column) pairs representing the assignment
     * @param num_rows Number of rows in the original problem
     * @param num_cols Number of columns in the original problem
     * @return True if the assignment is valid (no duplicates, within bounds)
     */
    static bool isValidAssignment(
        const std::vector<std::pair<int, int>>& assignment,
        int num_rows,
        int num_cols);

private:
    static constexpr double LARGE_VALUE = 1e9; ///< Large value for padding and infeasible assignments

    /**
     * @brief Core Hungarian algorithm implementation
     * @param cost_matrix Square cost matrix (will be modified)
     * @return Vector where assignment[i] = j means row i is assigned to column j
     */
    static std::vector<int> hungarianMethod(common::MatrixXd cost_matrix);

    /**
     * @brief Find maximal assignment using current zeros
     * @param cost_matrix The cost matrix
     * @param assignment Current assignment (will be modified)
     * @param row_covered Row covering status (will be modified)
     * @param col_covered Column covering status (will be modified)
     * @return Number of assignments found
     */
    static int findMaximalAssignment(
        const common::MatrixXd& cost_matrix,
        std::vector<int>& assignment,
        std::vector<bool>& row_covered,
        std::vector<bool>& col_covered);

    /**
     * @brief Find augmenting path for maximum bipartite matching
     * @param cost_matrix The cost matrix
     * @param row Starting row
     * @param assignment Current assignment (will be modified)
     * @param visited_cols Visited columns tracker (will be modified)
     * @return True if augmenting path was found
     */
    static bool findAugmentingPath(
        const common::MatrixXd& cost_matrix,
        int row,
        std::vector<int>& assignment,
        std::vector<bool>& visited_cols);

    /**
     * @brief Mark alternating path for minimum vertex cover
     * @param cost_matrix The cost matrix
     * @param row Starting row
     * @param assignment Current assignment
     * @param visited_rows Visited rows tracker (will be modified)
     * @param col_covered Column covering status
     */
    static void markAlternatingPath(
        const common::MatrixXd& cost_matrix,
        int row,
        const std::vector<int>& assignment,
        std::vector<bool>& visited_rows,
        std::vector<bool>& col_covered);

    /**
     * @brief Find minimum uncovered value in the matrix
     * @param cost_matrix The cost matrix
     * @param row_covered Row covering status
     * @param col_covered Column covering status
     * @return Minimum uncovered value
     */
    static double findMinUncovered(
        const common::MatrixXd& cost_matrix,
        const std::vector<bool>& row_covered,
        const std::vector<bool>& col_covered);
};

} // namespace association
} // namespace processing
} // namespace radar