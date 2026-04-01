#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace laplace_pathfinder
{

std::vector<std::vector<double>> load_npy_float64(const std::string & path, int & rows, int & cols);
std::vector<std::vector<int64_t>> load_npy_int64(const std::string & path, int & rows, int & cols);
void save_npy_float64(const std::string & path, const std::vector<std::vector<double>> & data);
bool write_csv_float_matrix(const std::string & path, const std::vector<std::vector<double>> & data);
bool write_csv_bool_matrix(const std::string & path, const std::vector<std::vector<bool>> & data);

}  // namespace laplace_pathfinder
