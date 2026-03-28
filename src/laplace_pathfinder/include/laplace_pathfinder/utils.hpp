#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace laplace_pathfinder
{

std::vector<std::vector<double>> load_npy_float64(const std::string & path, int & rows, int & cols);
std::vector<std::vector<int64_t>> load_npy_int64(const std::string & path, int & rows, int & cols);

}  // namespace laplace_pathfinder
