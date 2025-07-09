#include "association/hungarian_algorithm.hpp"
#include <algorithm>
#include <limits>
#include <vector>

namespace radar {
namespace processing {
namespace association {

HungarianAlgorithm::AssignmentResult HungarianAlgorithm::solve(const common::MatrixXd& cost_matrix) {
    AssignmentResult result;
    
    if (cost_matrix.rows() == 0 || cost_matrix.cols() == 0) {
        result.is_valid = false;
        result.total_cost = std::numeric_limits<double>::infinity();
        return result;
    }
    
    // Make the matrix square by padding with high costs
    int max_dim = std::max(cost_matrix.rows(), cost_matrix.cols());
    common::MatrixXd padded_matrix = common::MatrixXd::Constant(max_dim, max_dim, LARGE_VALUE);
    
    // Copy original cost matrix to top-left corner
    padded_matrix.block(0, 0, cost_matrix.rows(), cost_matrix.cols()) = cost_matrix;
    
    // Solve the assignment problem
    auto assignment = hungarianMethod(padded_matrix);
    
    // Extract valid assignments (not padded)
    result.assignment.clear();
    result.total_cost = 0.0;
    result.is_valid = true;
    
    for (int i = 0; i < static_cast<int>(assignment.size()); ++i) {
        int j = assignment[i];
        
        // Skip padded assignments
        if (i < cost_matrix.rows() && j < cost_matrix.cols() && 
            cost_matrix(i, j) < LARGE_VALUE) {
            result.assignment.emplace_back(i, j);
            result.total_cost += cost_matrix(i, j);
        }
    }
    
    return result;
}

std::vector<int> HungarianAlgorithm::hungarianMethod(common::MatrixXd cost_matrix) {
    int n = cost_matrix.rows();
    
    // Step 1: Subtract row minima
    for (int i = 0; i < n; ++i) {
        double row_min = cost_matrix.row(i).minCoeff();
        if (row_min < LARGE_VALUE) {
            cost_matrix.row(i).array() -= row_min;
        }
    }
    
    // Step 2: Subtract column minima
    for (int j = 0; j < n; ++j) {
        double col_min = cost_matrix.col(j).minCoeff();
        if (col_min < LARGE_VALUE) {
            cost_matrix.col(j).array() -= col_min;
        }
    }
    
    // Initialize tracking arrays
    std::vector<int> assignment(n, -1);  // assignment[i] = j means row i assigned to col j
    std::vector<bool> row_covered(n, false);
    std::vector<bool> col_covered(n, false);
    
    // Step 3: Cover zeros with minimum number of lines
    int num_covered_lines = 0;
    
    while (true) {
        // Try to find maximum assignment using current zeros
        assignment.assign(n, -1);
        std::fill(row_covered.begin(), row_covered.end(), false);
        std::fill(col_covered.begin(), col_covered.end(), false);
        
        int num_assignments = findMaximalAssignment(cost_matrix, assignment, row_covered, col_covered);
        
        if (num_assignments >= n) {
            // Found complete assignment
            break;
        }
        
        // Step 4: Find minimum uncovered value and adjust matrix
        double min_uncovered = findMinUncovered(cost_matrix, row_covered, col_covered);
        
        if (min_uncovered >= LARGE_VALUE) {
            // No valid assignment possible
            assignment.assign(n, -1);
            break;
        }
        
        // Subtract min_uncovered from uncovered elements
        // Add min_uncovered to elements covered by both row and column
        for (int i = 0; i < n; ++i) {
            for (int j = 0; j < n; ++j) {
                if (!row_covered[i] && !col_covered[j]) {
                    // Uncovered element
                    cost_matrix(i, j) -= min_uncovered;
                } else if (row_covered[i] && col_covered[j]) {
                    // Doubly covered element
                    cost_matrix(i, j) += min_uncovered;
                }
                // Singly covered elements remain unchanged
            }
        }
    }
    
    return assignment;
}

int HungarianAlgorithm::findMaximalAssignment(
    const common::MatrixXd& cost_matrix,
    std::vector<int>& assignment,
    std::vector<bool>& row_covered,
    std::vector<bool>& col_covered) {
    
    int n = cost_matrix.rows();
    
    // Use augmenting path method to find maximum assignment
    for (int i = 0; i < n; ++i) {
        std::vector<bool> visited_cols(n, false);
        findAugmentingPath(cost_matrix, i, assignment, visited_cols);
    }
    
    // Count assignments and mark covered rows/columns
    int num_assignments = 0;
    std::fill(row_covered.begin(), row_covered.end(), false);
    std::fill(col_covered.begin(), col_covered.end(), false);
    
    for (int i = 0; i < n; ++i) {
        if (assignment[i] != -1) {
            num_assignments++;
            row_covered[i] = true;
            col_covered[assignment[i]] = true;
        }
    }
    
    // For unassigned rows, use minimum vertex cover approach
    std::vector<bool> visited_rows(n, false);
    
    for (int i = 0; i < n; ++i) {
        if (assignment[i] == -1) {
            markAlternatingPath(cost_matrix, i, assignment, visited_rows, col_covered);
        }
    }
    
    // Update row covering based on visited rows
    for (int i = 0; i < n; ++i) {
        if (!visited_rows[i] && assignment[i] == -1) {
            row_covered[i] = false;
        } else if (visited_rows[i]) {
            row_covered[i] = false;
        }
    }
    
    return num_assignments;
}

bool HungarianAlgorithm::findAugmentingPath(
    const common::MatrixXd& cost_matrix,
    int row,
    std::vector<int>& assignment,
    std::vector<bool>& visited_cols) {
    
    int n = cost_matrix.rows();
    
    for (int col = 0; col < n; ++col) {
        if (cost_matrix(row, col) == 0.0 && !visited_cols[col]) {
            visited_cols[col] = true;
            
            // Find which row is assigned to this column
            int assigned_row = -1;
            for (int k = 0; k < n; ++k) {
                if (assignment[k] == col) {
                    assigned_row = k;
                    break;
                }
            }
            
            if (assigned_row == -1) {
                // Found augmenting path - unassigned column
                assignment[row] = col;
                return true;
            } else if (findAugmentingPath(cost_matrix, assigned_row, assignment, visited_cols)) {
                // Found augmenting path through assigned row
                assignment[row] = col;
                return true;
            }
        }
    }
    
    return false;
}

void HungarianAlgorithm::markAlternatingPath(
    const common::MatrixXd& cost_matrix,
    int row,
    const std::vector<int>& assignment,
    std::vector<bool>& visited_rows,
    std::vector<bool>& col_covered) {
    
    int n = cost_matrix.rows();
    
    if (visited_rows[row]) {
        return;
    }
    
    visited_rows[row] = true;
    
    for (int col = 0; col < n; ++col) {
        if (cost_matrix(row, col) == 0.0 && col_covered[col]) {
            // Find row assigned to this column
            for (int k = 0; k < n; ++k) {
                if (assignment[k] == col) {
                    markAlternatingPath(cost_matrix, k, assignment, visited_rows, col_covered);
                    break;
                }
            }
        }
    }
}

double HungarianAlgorithm::findMinUncovered(
    const common::MatrixXd& cost_matrix,
    const std::vector<bool>& row_covered,
    const std::vector<bool>& col_covered) {
    
    double min_value = LARGE_VALUE;
    int n = cost_matrix.rows();
    
    for (int i = 0; i < n; ++i) {
        if (!row_covered[i]) {
            for (int j = 0; j < n; ++j) {
                if (!col_covered[j]) {
                    min_value = std::min(min_value, cost_matrix(i, j));
                }
            }
        }
    }
    
    return min_value;
}

HungarianAlgorithm::AssignmentResult HungarianAlgorithm::solveMaximization(const common::MatrixXd& profit_matrix) {
    // Convert maximization to minimization by negating values
    double max_value = profit_matrix.maxCoeff();
    common::MatrixXd cost_matrix = (max_value * common::MatrixXd::Ones(profit_matrix.rows(), profit_matrix.cols())) - profit_matrix;
    
    auto result = solve(cost_matrix);
    
    // Convert cost back to profit
    if (result.is_valid) {
        result.total_cost = 0.0;
        for (const auto& assignment : result.assignment) {
            result.total_cost += profit_matrix(assignment.first, assignment.second);
        }
    }
    
    return result;
}

double HungarianAlgorithm::calculateAssignmentCost(
    const common::MatrixXd& cost_matrix,
    const std::vector<std::pair<int, int>>& assignment) {
    
    double total_cost = 0.0;
    
    for (const auto& pair : assignment) {
        if (pair.first >= 0 && pair.first < cost_matrix.rows() &&
            pair.second >= 0 && pair.second < cost_matrix.cols()) {
            total_cost += cost_matrix(pair.first, pair.second);
        }
    }
    
    return total_cost;
}

bool HungarianAlgorithm::isValidAssignment(
    const std::vector<std::pair<int, int>>& assignment,
    int num_rows,
    int num_cols) {
    
    std::vector<bool> used_rows(num_rows, false);
    std::vector<bool> used_cols(num_cols, false);
    
    for (const auto& pair : assignment) {
        int row = pair.first;
        int col = pair.second;
        
        // Check bounds
        if (row < 0 || row >= num_rows || col < 0 || col >= num_cols) {
            return false;
        }
        
        // Check for duplicates
        if (used_rows[row] || used_cols[col]) {
            return false;
        }
        
        used_rows[row] = true;
        used_cols[col] = true;
    }
    
    return true;
}

} // namespace association
} // namespace processing
} // namespace radar