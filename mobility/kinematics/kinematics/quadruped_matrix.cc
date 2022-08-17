// Copyright (c) Virtuous. Licensed under the MIT license.
// See LICENSE.md in the project root for license information.

#include "kinematics/quadruped_matrix.h"

#include <vector>
#include <math.h>
#include <iomanip>
#include <stdexcept>

namespace quadruped {

vector<vector<double>> Matrix::dot_product(vector<vector<double>> m1, vector<vector<double>> m2) {
  if(!(m1.size() == m2.size()))
    throw std::invalid_argument("Dot product not compatible.");

  vector<vector<double>> m = {{0, 0, 0, 0,}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}};
  double result = 0;
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      for (int h = 0; h < 4; h++) {
        result += m1[i][h] * m2[h][j];
      }
      m[i][j] = result;
      result = 0;
    }
  }
  return m;
}

// Might be funny but it's only when I finished theses functions that I realized that they only works for 2x2 matrices.

double get_determinant(const vector<vector<double>> vect) {
    if(vect.size() != vect[0].size()) {
        throw runtime_error("Matrix is not quadratic");
    } 
    size_t dimension = vect.size();

    if(dimension == 0) {
        return 1;
    }

    if(dimension == 1) {
        return vect[0][0];
    }

    // Formula for 2x2-matrix
    if(dimension == 2) {
        return vect[0][0] * vect[1][1] - vect[0][1] * vect[1][0];
    }

    double result = 0;
    int sign = 1;
    for(int i = 0; i < dimension; i++) {

        //Submatrix
        vector<vector<double>> subVect(dimension - 1, vector<double> (dimension - 1));
        for(int m = 1; m < dimension; m++) {
            int z = 0;
            for(int n = 0; n < dimension; n++) {
                if(n != i) {
                    subVect[m-1][z] = vect[m][n];
                    z++;
                }
            }
        }

        //recursive call
        result = result + sign * vect[0][i] * get_determinant(subVect);
        sign = -sign;
    }

    return result;
}

vector<vector<double>> get_transpose(const vector<vector<double>> matrix1) {

    //Transpose-matrix: height = width(matrix), width = height(matrix)
    vector<vector<double>> solution(matrix1[0].size(), vector<double> (matrix1.size()));

    //Filling solution-matrix
    for(size_t i = 0; i < matrix1.size(); i++) {
        for(size_t j = 0; j < matrix1[0].size(); j++) {
            solution[j][i] = matrix1[i][j];
        }
    }
    return solution;
}

vector<vector<double>> get_co_factor(const vector<vector<double>> vect) {
    if(vect.size() != vect[0].size()) {
        throw runtime_error("Matrix is not quadratic");
    } 

    vector<vector<double>> solution(vect.size(), vector<double> (vect.size()));
    vector<vector<double>> subVect(vect.size() - 1, vector<double> (vect.size() - 1));

    for(size_t i = 0; i < vect.size(); i++) {
        for(size_t j = 0; j < vect[0].size(); j++) {

            int p = 0;
            for(size_t x = 0; x < vect.size(); x++) {
                if(x == i) {
                    continue;
                }
                int q = 0;

                for(size_t y = 0; y < vect.size(); y++) {
                    if(y == j) {
                        continue;
                    }

                    subVect[p][q] = vect[x][y];
                    q++;
                }
                p++;
            }
            solution[i][j] = pow(-1, i + j) * get_determinant(subVect);
        }
    }
    return solution;
}

vector<vector<double>> Matrix::get_inverse(const vector<vector<double>> vect) {
    if(get_determinant(vect) == 0) {
        throw runtime_error("Determinant is 0");
    } 

    double d = 1.0/get_determinant(vect);
    vector<vector<double>> solution(vect.size(), vector<double> (vect.size()));

    for(size_t i = 0; i < vect.size(); i++) {
        for(size_t j = 0; j < vect.size(); j++) {
            solution[i][j] = vect[i][j]; 
        }
    }

    solution = get_transpose(get_co_factor(solution));

    for(size_t i = 0; i < vect.size(); i++) {
        for(size_t j = 0; j < vect.size(); j++) {
            solution[i][j] *= d;
        }
    }

    return solution;
}


}