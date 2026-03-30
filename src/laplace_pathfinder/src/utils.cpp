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


std::vector<std::vector<int64_t>> load_npy_int64(const std::string & path, int & rows, int & cols)
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

	// Require little-endian int64
	if (header.find("'<i8'") == std::string::npos &&
		header.find("\"<i8\"") == std::string::npos)
	{
		throw std::runtime_error(
			"Expected '<i8' (little-endian int64) in .npy header: " + path);
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

	// Read raw int64 values (row-major)
	const std::size_t n = static_cast<std::size_t>(rows) * cols;
	std::vector<int64_t> flat(n);
	f.read(
		reinterpret_cast<char *>(flat.data()),
		static_cast<std::streamsize>(n * sizeof(int64_t)));

	if (!f) {
		throw std::runtime_error("File ended prematurely while reading .npy data: " + path);
	}

	std::vector<std::vector<int64_t>> result(rows, std::vector<int64_t>(cols));
	for (int r = 0; r < rows; ++r) {
		for (int c = 0; c < cols; ++c) {
			result[r][c] = flat[static_cast<std::size_t>(r) * cols + c];
		}
	}
	return result;
}


void save_npy_float64(const std::string & path, const std::vector<std::vector<double>> & data)
{
	const int rows = static_cast<int>(data.size());
	const int cols = rows > 0 ? static_cast<int>(data[0].size()) : 0;

	// Build header string: must be padded so that (10 + header_len) % 64 == 0
	std::string hdr = "{'descr': '<f8', 'fortran_order': False, 'shape': (";
	hdr += std::to_string(rows) + ", " + std::to_string(cols) + "), }";
	// Pad with spaces to reach 64-byte alignment; terminate with '\n'
	// Total prefix before data: 6 (magic) + 1 (major) + 1 (minor) + 2 (hdr_len) = 10
	const std::size_t prefix_len = 10;
	const std::size_t raw_len = prefix_len + hdr.size() + 1;  // +1 for '\n'
	const std::size_t padded_len = ((raw_len + 63) / 64) * 64;
	hdr.append(padded_len - raw_len, ' ');
	hdr += '\n';

	std::ofstream f(path, std::ios::binary);
	if (!f) {
		throw std::runtime_error("Cannot open file for writing: " + path);
	}

	// Magic + version 1.0
	f.write("\x93NUMPY", 6);
	const uint8_t major = 1, minor = 0;
	f.write(reinterpret_cast<const char *>(&major), 1);
	f.write(reinterpret_cast<const char *>(&minor), 1);

	// Header length (uint16 little-endian)
	const uint16_t hdr_len = static_cast<uint16_t>(hdr.size());
	f.write(reinterpret_cast<const char *>(&hdr_len), 2);

	f.write(hdr.data(), static_cast<std::streamsize>(hdr.size()));

	// Data (row-major)
	for (int r = 0; r < rows; ++r) {
		f.write(
			reinterpret_cast<const char *>(data[r].data()),
			static_cast<std::streamsize>(cols * sizeof(double)));
	}

	if (!f) {
		throw std::runtime_error("Error writing .npy file: " + path);
	}
}

}  // namespace laplace_pathfinder
