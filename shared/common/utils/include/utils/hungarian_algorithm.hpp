#pragma once

#include "common/types.hpp"
#include <vector>

namespace radar {
namespace common {
namespace utils {

/**
 * @brief Hungarian Algorithm implementation for optimal assignment problems
 * 
 * This class implements the Hungarian algorithm (Kuhn-Munkres algorithm)
 * for solving the assignment problem in polynomial time. It finds the
 * minimum cost assignment in a bipartite graph.
 */
class HungarianAlgorithm {
public:
    /**
     * @brief Assignment result structure
     */
    struct AssignmentResult {
        std::vector<int> assignment;  // assignment[i] = j means row i assigned to column j
        double total_cost;            // Total cost of the assignment
        bool is_valid;               // Whether a valid assignment was found
    };

    /**
     * @brief Solve the assignment problem
     * @param cost_matrix Cost matrix (rows = workers, columns = tasks)
     * @return Assignment result with optimal assignment
     */
    static AssignmentResult solve(const MatrixXd& cost_matrix);

    /**
     * @brief Solve the assignment problem with maximum cost objective
     * @param profit_matrix Profit matrix (maximization problem)
     * @return Assignment result with optimal assignment
     */
    static AssignmentResult solveMaximization(const MatrixXd& profit_matrix);

private:
    /**
     * @brief Internal implementation of Hungarian algorithm
     * @param cost_matrix Square cost matrix
     * @return Assignment vector
     */
    static std::vector<int> hungarianSolve(MatrixXd cost_matrix);

    /**
     * @brief Step 1: Subtract row minimums
     * @param matrix Matrix to process
     */
    static void subtractRowMinimums(MatrixXd& matrix);

    /**
     * @brief Step 2: Subtract column minimums
     * @param matrix Matrix to process
     */
    static void subtractColumnMinimums(MatrixXd& matrix);

    /**
     * @brief Step 3: Cover all zeros with minimum number of lines
     * @param matrix Matrix to analyze
     * @param row_covered Row coverage array
     * @param col_covered Column coverage array
     * @return Number of covering lines
     */
    static int coverZeros(const MatrixXd& matrix, 
                         std::vector<bool>& row_covered,
                         std::vector<bool>& col_covered);

    /**
     * @brief Find the minimum uncovered value
     * @param matrix Matrix to search
     * @param row_covered Row coverage array
     * @param col_covered Column coverage array
     * @return Minimum uncovered value
     */
    static double findMinUncovered(const MatrixXd& matrix,
                                  const std::vector<bool>& row_covered,
                                  const std::vector<bool>& col_covered);

    /**
     * @brief Adjust matrix by subtracting minimum from uncovered elements
     * @param matrix Matrix to adjust
     * @param row_covered Row coverage array
     * @param col_covered Column coverage array
     * @param min_val Value to subtract
     */
    static void adjustMatrix(MatrixXd& matrix,
                           const std::vector<bool>& row_covered,
                           const std::vector<bool>& col_covered,
                           double min_val);

    /**
     * @brief Find an assignment using the covered zeros
     * @param matrix Matrix with zeros
     * @return Assignment vector (-1 if no assignment for that row)
     */
    static std::vector<int> findAssignment(const MatrixXd& matrix);

    /**
     * @brief Check if the assignment is complete
     * @param assignment Assignment vector
     * @return True if all rows are assigned
     */
    static bool isAssignmentComplete(const std::vector<int>& assignment);

    /**
     * @brief Calculate total cost of assignment
     * @param original_matrix Original cost matrix
     * @param assignment Assignment vector
     * @return Total cost
     */
    static double calculateTotalCost(const MatrixXd& original_matrix,
                                   const std::vector<int>& assignment);

    /**
     * @brief Make matrix square by padding with high costs
     * @param matrix Original matrix
     * @param pad_value Value to use for padding
     * @return Square matrix
     */
    static MatrixXd makeSquare(const MatrixXd& matrix, double pad_value = 1e9);

    /**
     * @brief Find zeros in the matrix
     * @param matrix Matrix to search
     * @return Vector of (row, col) pairs for zero locations
     */
    static std::vector<std::pair<int, int>> findZeros(const MatrixXd& matrix);

    /**
     * @brief Mark zeros for assignment using augmenting path
     * @param matrix Matrix with zeros
     * @param assignment Current assignment
     * @return Updated assignment
     */
    static std::vector<int> augmentAssignment(const MatrixXd& matrix,
                                            std::vector<int> assignment);
};

} // namespace utils
} // namespace common
} // namespace radar