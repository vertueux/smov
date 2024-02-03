// Copyright (c) Virtuous. Licensed under the GNU AFFERO GENERAL PUBLIC LICENSE.
// See LICENSE in the project root for license information.

#ifndef QUADRUPED_MATRIX_H
#define QUADRUPED_MATRIX_H

#define N 4

#include <vector>

namespace quadruped {
  
class Matrix {
 public:
  static std::vector<std::vector<double>> dot_product(std::vector<std::vector<double>> m1, std::vector<std::vector<double>> m2);
  static std::vector<std::vector<double>> get_inverse(const std::vector<std::vector<double>> vect); // Only works with 2x2 matrices for now.
};

} // namespace quadruped

#endif // QUADRUPED_MATRIX_H
