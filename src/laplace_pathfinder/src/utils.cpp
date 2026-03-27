#include "laplace_pathfinder/utils.hpp"

#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace laplace_pathfinder
{

std::vector<std::vector<double>> load_npy_float64(const std::string & path, int & rows, int & cols)
{
	std::ifstream f(path, std::ios::binary);
	if (!f) {
		throw std::runtime_error("Cannot open .npy file: " + path);
	}

	// Validate magic "\x93NUMPY"
	char magic[6];
	f.read(magic, 6);
	const std::string npy_magic("\x93NUMPY", 6);
	if (std::string(magic, 6) != npy_magic) {
		throw std::runtime_error("Not a .npy file: " + path);
	}

	// Version
	uint8_t major = 0, minor = 0;
	f.read(reinterpret_cast<char *>(&major), 1);
	f.read(reinterpret_cast<char *>(&minor), 1);
	(void)minor;

	// Header length: 2 bytes for v1, 4 bytes for v2+
	uint32_t hdr_len = 0;
	if (major == 1) {
		uint16_t h16 = 0;
		f.read(reinterpret_cast<char *>(&h16), 2);
		hdr_len = h16;
	} else {
		f.read(reinterpret_cast<char *>(&hdr_len), 4);
	}

	std::string header(hdr_len, '\0');
	f.read(header.data(), hdr_len);

	// Require little-endian float64
	if (header.find("'<f8'") == std::string::npos &&
		header.find("\"<f8\"") == std::string::npos)
	{
		throw std::runtime_error(
			"Expected '<f8' (little-endian float64) in .npy header: " + path);
	}

	// Parse shape
	auto pos = header.find("'shape'");
	if (pos == std::string::npos) {
		throw std::runtime_error("Cannot find 'shape' in .npy header: " + path);
	}
	const auto open  = header.find('(', pos);
	const auto close = header.find(')', open);
	const std::string shape_str = header.substr(open + 1, close - open - 1);
	const auto comma = shape_str.find(',');
	rows = std::stoi(shape_str.substr(0, comma));
	cols = std::stoi(shape_str.substr(comma + 1));

	// Read raw doubles (row-major)
	const std::size_t n = static_cast<std::size_t>(rows) * cols;
	std::vector<double> flat(n);
	f.read(
		reinterpret_cast<char *>(flat.data()),
		static_cast<std::streamsize>(n * sizeof(double)));

	if (!f) {
		throw std::runtime_error("File ended prematurely while reading .npy data: " + path);
	}

	std::vector<std::vector<double>> result(rows, std::vector<double>(cols));
	for (int r = 0; r < rows; ++r) {
		for (int c = 0; c < cols; ++c) {
			result[r][c] = flat[static_cast<std::size_t>(r) * cols + c];
		}
	}
	return result;
}

}  // namespace laplace_pathfinder
