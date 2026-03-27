#pragma once

#include <string>
#include <vector>

namespace laplace_pathfinder
{

std::vector<std::vector<double>> load_npy_float64(const std::string & path, int & rows, int & cols);

}  // namespace laplace_pathfinder
