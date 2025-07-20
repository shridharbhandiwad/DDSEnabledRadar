#include "association/hungarian_algorithm.hpp"
#include <limits>
#include <algorithm>
#include <vector>

namespace radar {
namespace processing {
namespace association {

/**
 * @brief Implementation of the Hungarian Algorithm for solving assignment problems
 * 
 * The Hungarian algorithm (also known as the Munkres algorithm) solves the assignment
 * problem in polynomial time. It finds the minimum-cost perfect matching in a bipartite
 * graph, which is essential for optimal data association in multi-target tracking.
 * 
 * Algorithm Steps:
 * 1. Subtract row minimums from each row
 * 2. Subtract column minimums from each column  
 * 3. Cover all zeros with minimum number of lines
 * 4. If lines < matrix dimension, modify matrix and repeat
 * 5. Find optimal assignment from final matrix
 * 
 * Time Complexity: O(n³) where n is the matrix dimension
 * Space Complexity: O(n²) for the working matrix
 */

const double LARGE_VALUE = 1e9;

std::vector<std::pair<int, int>> HungarianAlgorithm::solve(const common::MatrixXd& cost_matrix) {
    if (cost_matrix.rows() == 0 || cost_matrix.cols() == 0) {
        return {};
    }
    
    // Create square matrix by padding with large values if necessary
    size_t max_dim = std::max(cost_matrix.rows(), cost_matrix.cols());
    common::MatrixXd padded_matrix = common::MatrixXd::Constant(max_dim, max_dim, LARGE_VALUE);
    
    // Copy original cost matrix to top-left corner
    for (size_t i = 0; i < cost_matrix.rows(); ++i) {
        for (size_t j = 0; j < cost_matrix.cols(); ++j) {
            padded_matrix[i][j] = cost_matrix[i][j];
        }
    }
    
    return solveSquareMatrix(padded_matrix);
}

std::vector<std::pair<int, int>> HungarianAlgorithm::solveSquareMatrix(common::MatrixXd& matrix) {
    size_t n = matrix.rows();
    
    /**
     * Step 1: Subtract the minimum value in each row from all elements in that row
     * This operation preserves the optimal assignment while reducing the cost matrix
     */
    subtractRowMinimums(matrix);
    
    /**
     * Step 2: Subtract the minimum value in each column from all elements in that column  
     * This further reduces the matrix while maintaining optimality
     */
    subtractColumnMinimums(matrix);
    
    /**
     * Step 3: Iteratively cover zeros and modify matrix until optimal solution is found
     * The algorithm alternates between covering zeros with lines and modifying the matrix
     * until the number of covering lines equals the matrix dimension
     */
    std::vector<bool> row_covered(n, false);
    std::vector<bool> col_covered(n, false);
    std::vector<std::vector<bool>> starred_zeros(n, std::vector<bool>(n, false));
    std::vector<std::vector<bool>> primed_zeros(n, std::vector<bool>(n, false));
    
    // Find initial starred zeros (one per row and column)
    findInitialStarredZeros(matrix, starred_zeros);
    
    while (true) {
        // Cover columns that contain starred zeros
        coverStarredColumns(starred_zeros, col_covered);
        
        /**
         * Count the number of covered columns
         * If all columns are covered, we have found the optimal assignment
         */
        int covered_columns = std::count(col_covered.begin(), col_covered.end(), true);
        
        if (covered_columns == static_cast<int>(n)) {
            // Optimal assignment found
            break;
        }
        
        /**
         * Find uncovered zeros and prime them
         * If a primed zero is in a row with no starred zero, we can improve the solution
         */
        while (true) {
            auto uncovered_zero = findUncoveredZero(matrix, row_covered, col_covered);
            
            if (uncovered_zero.first == -1) {
                // No uncovered zeros found, modify the matrix
                modifyMatrix(matrix, row_covered, col_covered);
                continue;
            }
            
            // Prime the uncovered zero
            primed_zeros[uncovered_zero.first][uncovered_zero.second] = true;
            
            // Check if there's a starred zero in the same row
            int starred_col = findStarredZeroInRow(starred_zeros, uncovered_zero.first);
            
            if (starred_col != -1) {
                /**
                 * Cover the row and uncover the column containing the starred zero
                 * This maintains the invariant while allowing us to continue searching
                 */
                row_covered[uncovered_zero.first] = true;
                col_covered[starred_col] = false;
            } else {
                /**
                 * Found an augmenting path - construct new starring
                 * This improves the current solution by increasing the number
                 * of assignments by one.
                 */
                constructAugmentingPath(starred_zeros, primed_zeros, uncovered_zero.first, uncovered_zero.second);
                
                // Clear covers and primed zeros for next iteration
                clearCovers(row_covered, col_covered);
                clearPrimedZeros(primed_zeros);
                break;
            }
        }
    }
    
    // Extract the final assignment from starred zeros
    return extractAssignment(starred_zeros);
}

void HungarianAlgorithm::subtractRowMinimums(common::MatrixXd& matrix) {
    /**
     * For each row, find the minimum value and subtract it from all elements in that row
     * This operation reduces the matrix while preserving the optimal assignment structure
     */
    for (size_t i = 0; i < matrix.rows(); ++i) {
        double min_val = *std::min_element(matrix[i].begin(), matrix[i].end());
        for (size_t j = 0; j < matrix.cols(); ++j) {
            matrix[i][j] -= min_val;
        }
    }
}

void HungarianAlgorithm::subtractColumnMinimums(common::MatrixXd& matrix) {
    /**
     * For each column, find the minimum value and subtract it from all elements in that column
     * This step completes the initial matrix reduction phase
     */
    for (size_t j = 0; j < matrix.cols(); ++j) {
        double min_val = matrix[0][j];
        for (size_t i = 1; i < matrix.rows(); ++i) {
            min_val = std::min(min_val, matrix[i][j]);
        }
        for (size_t i = 0; i < matrix.rows(); ++i) {
            matrix[i][j] -= min_val;
        }
    }
}

void HungarianAlgorithm::findInitialStarredZeros(
    const common::MatrixXd& matrix,
    std::vector<std::vector<bool>>& starred_zeros) {
    
    /**
     * Star zeros to create an initial partial assignment
     * We star a zero if there are no other starred zeros in its row or column
     * This creates a maximal matching as a starting point
     */
    size_t n = matrix.rows();
    std::vector<bool> row_has_star(n, false);
    std::vector<bool> col_has_star(n, false);
    
    for (size_t i = 0; i < n; ++i) {
        for (size_t j = 0; j < n; ++j) {
            if (matrix[i][j] == 0.0 && !row_has_star[i] && !col_has_star[j]) {
                starred_zeros[i][j] = true;
                row_has_star[i] = true;
                col_has_star[j] = true;
            }
        }
    }
}

void HungarianAlgorithm::coverStarredColumns(
    const std::vector<std::vector<bool>>& starred_zeros,
    std::vector<bool>& col_covered) {
    
    /**
     * Cover all columns that contain starred zeros
     * This is part of the line covering step to check if we have enough assignments
     */
    size_t n = starred_zeros.size();
    std::fill(col_covered.begin(), col_covered.end(), false);
    
    for (size_t i = 0; i < n; ++i) {
        for (size_t j = 0; j < n; ++j) {
            if (starred_zeros[i][j]) {
                col_covered[j] = true;
            }
        }
    }
}

std::pair<int, int> HungarianAlgorithm::findUncoveredZero(
    const common::MatrixXd& matrix,
    const std::vector<bool>& row_covered,
    const std::vector<bool>& col_covered) {
    
    /**
     * Find a zero that is not covered by any line
     * Uncovered zeros represent potential improvements to the current assignment
     */
    size_t n = matrix.rows();
    
    for (size_t i = 0; i < n; ++i) {
        if (row_covered[i]) continue;
        
        for (size_t j = 0; j < n; ++j) {
            if (col_covered[j]) continue;
            
            if (matrix[i][j] == 0.0) {
                return {static_cast<int>(i), static_cast<int>(j)};
            }
        }
    }
    
    return {-1, -1}; // No uncovered zero found
}

void HungarianAlgorithm::modifyMatrix(
    common::MatrixXd& matrix,
    const std::vector<bool>& row_covered,
    const std::vector<bool>& col_covered) {
    
    /**
     * Modify the matrix to create new zeros while maintaining optimality
     * 
     * Find the minimum uncovered value, then:
     * - Subtract it from all uncovered elements (creates new zeros)
     * - Add it to all doubly covered elements (prevents negative values)
     * 
     * This modification preserves the optimal assignment while creating
     * new opportunities for improvement
     */
    size_t n = matrix.rows();
    
    // Find minimum uncovered value
    double min_uncovered = std::numeric_limits<double>::max();
    for (size_t i = 0; i < n; ++i) {
        if (row_covered[i]) continue;
        
        for (size_t j = 0; j < n; ++j) {
            if (col_covered[j]) continue;
            
            min_uncovered = std::min(min_uncovered, matrix[i][j]);
        }
    }
    
    // Modify matrix elements based on their coverage status
    for (size_t i = 0; i < n; ++i) {
        for (size_t j = 0; j < n; ++j) {
            if (!row_covered[i] && !col_covered[j]) {
                // Uncovered: subtract minimum
                matrix[i][j] -= min_uncovered;
            } else if (row_covered[i] && col_covered[j]) {
                // Doubly covered: add minimum
                matrix[i][j] += min_uncovered;
            }
            // Singly covered elements remain unchanged
        }
    }
}

int HungarianAlgorithm::findStarredZeroInRow(
    const std::vector<std::vector<bool>>& starred_zeros,
    int row) {
    
    /**
     * Find the column containing a starred zero in the given row
     * Returns -1 if no starred zero exists in the row
     */
    size_t n = starred_zeros.size();
    
    for (size_t j = 0; j < n; ++j) {
        if (starred_zeros[row][j]) {
            return static_cast<int>(j);
        }
    }
    
    return -1;
}

void HungarianAlgorithm::constructAugmentingPath(
    std::vector<std::vector<bool>>& starred_zeros,
    std::vector<std::vector<bool>>& primed_zeros,
    int start_row,
    int start_col) {
    
    /**
     * Construct an augmenting path to improve the current assignment
     * 
     * An augmenting path alternates between primed and starred zeros,
     * starting with a primed zero and ending with a primed zero.
     * By flipping the starring along this path, we increase the number
     * of assignments by one.
     * 
     * The path construction follows this pattern:
     * Primed(i,j) -> Starred(i,k) -> Primed(l,k) -> Starred(l,m) -> ...
     */
    std::vector<std::pair<int, int>> path;
    path.emplace_back(start_row, start_col);
    
    size_t n = starred_zeros.size();
    
    while (true) {
        // Find starred zero in the current column
        int current_col = path.back().second;
        int starred_row = -1;
        
        for (size_t i = 0; i < n; ++i) {
            if (starred_zeros[i][current_col]) {
                starred_row = static_cast<int>(i);
                break;
            }
        }
        
        if (starred_row == -1) {
            // No starred zero found in this column, path is complete
            break;
        }
        
        path.emplace_back(starred_row, current_col);
        
        // Find primed zero in the starred zero's row
        int primed_col = -1;
        for (size_t j = 0; j < n; ++j) {
            if (primed_zeros[starred_row][j]) {
                primed_col = static_cast<int>(j);
                break;
            }
        }
        
        if (primed_col == -1) {
            // This should not happen in a correct implementation
            break;
        }
        
        path.emplace_back(starred_row, primed_col);
    }
    
    /**
     * Flip the starring along the augmenting path
     * - Unstar all starred zeros in the path
     * - Star all primed zeros in the path
     */
    for (size_t i = 0; i < path.size(); ++i) {
        int row = path[i].first;
        int col = path[i].second;
        
        if (i % 2 == 0) {
            // Even indices: star the primed zeros
            starred_zeros[row][col] = true;
        } else {
            // Odd indices: unstar the starred zeros
            starred_zeros[row][col] = false;
        }
    }
}

void HungarianAlgorithm::clearCovers(
    std::vector<bool>& row_covered,
    std::vector<bool>& col_covered) {
    
    /**
     * Clear all row and column covers for the next iteration
     */
    std::fill(row_covered.begin(), row_covered.end(), false);
    std::fill(col_covered.begin(), col_covered.end(), false);
}

void HungarianAlgorithm::clearPrimedZeros(std::vector<std::vector<bool>>& primed_zeros) {
    /**
     * Clear all primed zeros for the next iteration
     */
    for (auto& row : primed_zeros) {
        std::fill(row.begin(), row.end(), false);
    }
}

std::vector<std::pair<int, int>> HungarianAlgorithm::extractAssignment(
    const std::vector<std::vector<bool>>& starred_zeros) {
    
    /**
     * Extract the final optimal assignment from the starred zeros
     * Each starred zero represents an assignment in the optimal solution
     */
    std::vector<std::pair<int, int>> assignment;
    size_t n = starred_zeros.size();
    
    for (size_t i = 0; i < n; ++i) {
        for (size_t j = 0; j < n; ++j) {
            if (starred_zeros[i][j]) {
                assignment.emplace_back(static_cast<int>(i), static_cast<int>(j));
            }
        }
    }
    
    return assignment;
}

double HungarianAlgorithm::calculateTotalCost(
    const common::MatrixXd& cost_matrix,
    const std::vector<std::pair<int, int>>& assignment) {
    
    /**
     * Calculate the total cost of an assignment
     * This is useful for validating the optimality of the solution
     */
    double total_cost = 0.0;
    
    for (const auto& pair : assignment) {
        if (pair.first < static_cast<int>(cost_matrix.rows()) && 
            pair.second < static_cast<int>(cost_matrix.cols())) {
            total_cost += cost_matrix[pair.first][pair.second];
        }
    }
    
    return total_cost;
}

std::vector<std::pair<int, int>> HungarianAlgorithm::solveMaximization(const common::MatrixXd& profit_matrix) {
    /**
     * Solve maximization assignment problem by converting to minimization
     * 
     * The Hungarian algorithm solves minimization problems. For maximization:
     * 1. Find the maximum value in the profit matrix
     * 2. Subtract each element from this maximum to create a cost matrix
     * 3. Solve the resulting minimization problem
     * 
     * This transformation preserves the optimal assignment structure.
     */
    if (profit_matrix.rows() == 0 || profit_matrix.cols() == 0) {
        return {};
    }
    
    // Find maximum value in the matrix
    double max_value = profit_matrix[0][0];
    for (size_t i = 0; i < profit_matrix.rows(); ++i) {
        for (size_t j = 0; j < profit_matrix.cols(); ++j) {
            max_value = std::max(max_value, profit_matrix[i][j]);
        }
    }
    
    // Convert to cost matrix: cost = max_value - profit
    common::MatrixXd cost_matrix = (max_value * common::MatrixXd::Ones(profit_matrix.rows(), profit_matrix.cols())) - profit_matrix;
    
    // Solve the minimization problem
    return solve(cost_matrix);
}

} // namespace association
} // namespace processing
} // namespace radar