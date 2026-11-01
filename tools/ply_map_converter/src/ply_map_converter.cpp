#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace fs = std::filesystem;

namespace
{

constexpr std::uint8_t kPgmOccupied = 0;
constexpr std::uint8_t kPgmUnknown = 205;
constexpr std::uint8_t kPgmFree = 254;

enum class ScalarType
{
  Int8,
  UInt8,
  Int16,
  UInt16,
  Int32,
  UInt32,
  Float32,
  Float64,
};

struct PlyProperty
{
  std::string name;
  ScalarType type{};
  bool is_list = false;
  ScalarType list_count_type{};
  ScalarType list_value_type{};
};

struct PlyElement
{
  std::string name;
  std::uint64_t count = 0;
  std::vector<PlyProperty> properties;
};

struct PlyHeader
{
  std::string format;
  std::vector<PlyElement> elements;
};

struct Options
{
  fs::path input;
  std::string map_name;
  fs::path bringup_maps_dir = fs::path("src") / "bringup" / "maps";
  double resolution = 0.05;
  double padding = 0.5;
  double pcd_voxel_size = 0.05;
  double free_min_z = -0.05;
  double free_max_z = 0.08;
  double occupied_min_z = 0.15;
  double occupied_max_z = 1.70;
  bool overwrite = false;
};

struct Bounds
{
  double min_x = std::numeric_limits<double>::max();
  double min_y = std::numeric_limits<double>::max();
  double min_z = std::numeric_limits<double>::max();
  double max_x = std::numeric_limits<double>::lowest();
  double max_y = std::numeric_limits<double>::lowest();
  double max_z = std::numeric_limits<double>::lowest();

  void include(const pcl::PointXYZI & point)
  {
    min_x = std::min<double>(min_x, point.x);
    min_y = std::min<double>(min_y, point.y);
    min_z = std::min<double>(min_z, point.z);
    max_x = std::max<double>(max_x, point.x);
    max_y = std::max<double>(max_y, point.y);
    max_z = std::max<double>(max_z, point.z);
  }
};

struct OutputPaths
{
  fs::path root;
  fs::path pcd;
  fs::path pgm;
  fs::path yaml;
};

std::string trim_copy(std::string value)
{
  while (!value.empty() && std::isspace(static_cast<unsigned char>(value.back()))) {
    value.pop_back();
  }
  std::size_t first = 0;
  while (first < value.size() && std::isspace(static_cast<unsigned char>(value[first]))) {
    ++first;
  }
  return value.substr(first);
}

std::string to_lower_copy(std::string value)
{
  std::transform(
    value.begin(),
    value.end(),
    value.begin(),
    [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return value;
}

ScalarType parse_scalar_type(const std::string & raw_type)
{
  const auto type = to_lower_copy(raw_type);
  if (type == "char" || type == "int8") {
    return ScalarType::Int8;
  }
  if (type == "uchar" || type == "uint8") {
    return ScalarType::UInt8;
  }
  if (type == "short" || type == "int16") {
    return ScalarType::Int16;
  }
  if (type == "ushort" || type == "uint16") {
    return ScalarType::UInt16;
  }
  if (type == "int" || type == "int32") {
    return ScalarType::Int32;
  }
  if (type == "uint" || type == "uint32") {
    return ScalarType::UInt32;
  }
  if (type == "float" || type == "float32") {
    return ScalarType::Float32;
  }
  if (type == "double" || type == "float64") {
    return ScalarType::Float64;
  }
  throw std::runtime_error("unsupported PLY scalar type: " + raw_type);
}

std::size_t scalar_size(const ScalarType type)
{
  switch (type) {
    case ScalarType::Int8:
    case ScalarType::UInt8:
      return 1;
    case ScalarType::Int16:
    case ScalarType::UInt16:
      return 2;
    case ScalarType::Int32:
    case ScalarType::UInt32:
    case ScalarType::Float32:
      return 4;
    case ScalarType::Float64:
      return 8;
  }
  throw std::runtime_error("invalid scalar type");
}

template<typename T>
T read_little_endian(const std::uint8_t * data)
{
  T value{};
  std::memcpy(&value, data, sizeof(T));
  return value;
}

double scalar_to_double(const std::uint8_t * data, const ScalarType type)
{
  switch (type) {
    case ScalarType::Int8:
      return static_cast<double>(read_little_endian<std::int8_t>(data));
    case ScalarType::UInt8:
      return static_cast<double>(read_little_endian<std::uint8_t>(data));
    case ScalarType::Int16:
      return static_cast<double>(read_little_endian<std::int16_t>(data));
    case ScalarType::UInt16:
      return static_cast<double>(read_little_endian<std::uint16_t>(data));
    case ScalarType::Int32:
      return static_cast<double>(read_little_endian<std::int32_t>(data));
    case ScalarType::UInt32:
      return static_cast<double>(read_little_endian<std::uint32_t>(data));
    case ScalarType::Float32:
      return static_cast<double>(read_little_endian<float>(data));
    case ScalarType::Float64:
      return read_little_endian<double>(data);
  }
  throw std::runtime_error("invalid scalar type");
}

std::uint64_t scalar_to_uint64(const std::uint8_t * data, const ScalarType type)
{
  const auto value = scalar_to_double(data, type);
  if (value < 0.0) {
    throw std::runtime_error("negative PLY list count is invalid");
  }
  return static_cast<std::uint64_t>(value);
}

PlyHeader parse_ply_header(std::istream & input)
{
  std::string line;
  if (!std::getline(input, line) || trim_copy(line) != "ply") {
    throw std::runtime_error("input is not a PLY file");
  }

  PlyHeader header;
  PlyElement * current_element = nullptr;
  bool saw_end_header = false;
  while (std::getline(input, line)) {
    line = trim_copy(line);
    if (line.empty()) {
      continue;
    }

    std::istringstream stream(line);
    std::string token;
    stream >> token;

    if (token == "comment" || token == "obj_info") {
      continue;
    }
    if (token == "format") {
      stream >> header.format;
      continue;
    }
    if (token == "element") {
      PlyElement element;
      stream >> element.name >> element.count;
      if (element.name.empty() || !stream) {
        throw std::runtime_error("invalid PLY element line: " + line);
      }
      header.elements.push_back(std::move(element));
      current_element = &header.elements.back();
      continue;
    }
    if (token == "property") {
      if (!current_element) {
        throw std::runtime_error("PLY property appears before any element");
      }

      std::string maybe_list_or_type;
      stream >> maybe_list_or_type;
      if (maybe_list_or_type == "list") {
        std::string count_type;
        std::string value_type;
        std::string name;
        stream >> count_type >> value_type >> name;
        if (name.empty() || !stream) {
          throw std::runtime_error("invalid PLY list property line: " + line);
        }
        current_element->properties.push_back(
          PlyProperty{name, {}, true, parse_scalar_type(count_type), parse_scalar_type(value_type)});
      } else {
        std::string name;
        stream >> name;
        if (name.empty() || !stream) {
          throw std::runtime_error("invalid PLY property line: " + line);
        }
        current_element->properties.push_back(
          PlyProperty{name, parse_scalar_type(maybe_list_or_type), false, {}, {}});
      }
      continue;
    }
    if (token == "end_header") {
      saw_end_header = true;
      break;
    }
  }

  if (!saw_end_header) {
    throw std::runtime_error("PLY header is missing end_header");
  }
  if (header.format != "binary_little_endian" && header.format != "ascii") {
    throw std::runtime_error(
      "unsupported PLY format '" + header.format + "', expected binary_little_endian or ascii");
  }
  if (header.elements.empty()) {
    throw std::runtime_error("PLY file has no elements");
  }
  return header;
}

std::unordered_map<std::string, std::size_t> make_property_index(
  const std::vector<PlyProperty> & properties)
{
  std::unordered_map<std::string, std::size_t> index;
  for (std::size_t i = 0; i < properties.size(); ++i) {
    index.emplace(properties[i].name, i);
  }
  return index;
}

std::vector<std::size_t> scalar_offsets(const std::vector<PlyProperty> & properties)
{
  std::vector<std::size_t> offsets;
  offsets.reserve(properties.size());
  std::size_t offset = 0;
  for (const auto & property : properties) {
    if (property.is_list) {
      throw std::runtime_error("list properties are not supported on PLY vertices");
    }
    offsets.push_back(offset);
    offset += scalar_size(property.type);
  }
  return offsets;
}

std::size_t scalar_record_size(const std::vector<PlyProperty> & properties)
{
  std::size_t record_size = 0;
  for (const auto & property : properties) {
    if (property.is_list) {
      throw std::runtime_error("list properties are not supported on PLY vertices");
    }
    record_size += scalar_size(property.type);
  }
  return record_size;
}

void skip_binary_element(std::istream & input, const PlyElement & element)
{
  for (std::uint64_t i = 0; i < element.count; ++i) {
    for (const auto & property : element.properties) {
      if (!property.is_list) {
        input.ignore(static_cast<std::streamsize>(scalar_size(property.type)));
        continue;
      }
      std::array<std::uint8_t, 8> count_buffer{};
      const auto count_size = scalar_size(property.list_count_type);
      input.read(reinterpret_cast<char *>(count_buffer.data()), static_cast<std::streamsize>(count_size));
      if (!input) {
        throw std::runtime_error("unexpected EOF while skipping PLY list property");
      }
      const auto count = scalar_to_uint64(count_buffer.data(), property.list_count_type);
      input.ignore(static_cast<std::streamsize>(count * scalar_size(property.list_value_type)));
    }
    if (!input) {
      throw std::runtime_error("unexpected EOF while skipping PLY element");
    }
  }
}

float color_intensity(double red, double green, double blue)
{
  red = std::clamp(red, 0.0, 255.0);
  green = std::clamp(green, 0.0, 255.0);
  blue = std::clamp(blue, 0.0, 255.0);
  return static_cast<float>((red + green + blue) / 3.0);
}

pcl::PointCloud<pcl::PointXYZI>::Ptr read_binary_vertices(
  std::istream & input,
  const PlyElement & element,
  Bounds & bounds)
{
  const auto property_index = make_property_index(element.properties);
  for (const auto * required : {"x", "y", "z"}) {
    if (property_index.find(required) == property_index.end()) {
      throw std::runtime_error("PLY vertex is missing required property: " + std::string(required));
    }
  }

  const auto offsets = scalar_offsets(element.properties);
  const auto record_size = scalar_record_size(element.properties);
  std::vector<std::uint8_t> buffer(record_size);
  const auto x_index = property_index.at("x");
  const auto y_index = property_index.at("y");
  const auto z_index = property_index.at("z");
  const bool has_rgb =
    property_index.find("red") != property_index.end() &&
    property_index.find("green") != property_index.end() &&
    property_index.find("blue") != property_index.end();

  auto cloud = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
  cloud->points.reserve(static_cast<std::size_t>(element.count));

  for (std::uint64_t i = 0; i < element.count; ++i) {
    input.read(reinterpret_cast<char *>(buffer.data()), static_cast<std::streamsize>(buffer.size()));
    if (!input) {
      throw std::runtime_error("unexpected EOF while reading PLY vertices");
    }

    pcl::PointXYZI point;
    point.x = static_cast<float>(
      scalar_to_double(buffer.data() + offsets[x_index], element.properties[x_index].type));
    point.y = static_cast<float>(
      scalar_to_double(buffer.data() + offsets[y_index], element.properties[y_index].type));
    point.z = static_cast<float>(
      scalar_to_double(buffer.data() + offsets[z_index], element.properties[z_index].type));
    point.intensity = 0.0F;

    if (has_rgb) {
      const auto r_index = property_index.at("red");
      const auto g_index = property_index.at("green");
      const auto b_index = property_index.at("blue");
      point.intensity = color_intensity(
        scalar_to_double(buffer.data() + offsets[r_index], element.properties[r_index].type),
        scalar_to_double(buffer.data() + offsets[g_index], element.properties[g_index].type),
        scalar_to_double(buffer.data() + offsets[b_index], element.properties[b_index].type));
    }

    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
      continue;
    }
    bounds.include(point);
    cloud->push_back(point);
  }

  cloud->width = static_cast<std::uint32_t>(cloud->points.size());
  cloud->height = 1;
  cloud->is_dense = false;
  return cloud;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr read_ascii_vertices(
  std::istream & input,
  const PlyElement & element,
  Bounds & bounds)
{
  const auto property_index = make_property_index(element.properties);
  for (const auto & property : element.properties) {
    if (property.is_list) {
      throw std::runtime_error("list properties are not supported on PLY vertices");
    }
  }
  for (const auto * required : {"x", "y", "z"}) {
    if (property_index.find(required) == property_index.end()) {
      throw std::runtime_error("PLY vertex is missing required property: " + std::string(required));
    }
  }

  const auto x_index = property_index.at("x");
  const auto y_index = property_index.at("y");
  const auto z_index = property_index.at("z");
  const bool has_rgb =
    property_index.find("red") != property_index.end() &&
    property_index.find("green") != property_index.end() &&
    property_index.find("blue") != property_index.end();

  auto cloud = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
  cloud->points.reserve(static_cast<std::size_t>(element.count));

  std::string line;
  for (std::uint64_t i = 0; i < element.count; ++i) {
    if (!std::getline(input, line)) {
      throw std::runtime_error("unexpected EOF while reading ASCII PLY vertices");
    }
    std::istringstream stream(line);
    std::vector<double> values(element.properties.size(), 0.0);
    for (std::size_t p = 0; p < element.properties.size(); ++p) {
      stream >> values[p];
      if (!stream) {
        throw std::runtime_error("invalid ASCII PLY vertex row");
      }
    }

    pcl::PointXYZI point;
    point.x = static_cast<float>(values[x_index]);
    point.y = static_cast<float>(values[y_index]);
    point.z = static_cast<float>(values[z_index]);
    point.intensity = 0.0F;
    if (has_rgb) {
      point.intensity = color_intensity(
        values[property_index.at("red")],
        values[property_index.at("green")],
        values[property_index.at("blue")]);
    }

    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
      continue;
    }
    bounds.include(point);
    cloud->push_back(point);
  }

  cloud->width = static_cast<std::uint32_t>(cloud->points.size());
  cloud->height = 1;
  cloud->is_dense = false;
  return cloud;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr load_ply(const fs::path & path, Bounds & bounds)
{
  std::ifstream input(path, std::ios::binary);
  if (!input) {
    throw std::runtime_error("failed to open input PLY: " + path.string());
  }

  const auto header = parse_ply_header(input);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
  bool read_vertices = false;

  for (const auto & element : header.elements) {
    if (element.name == "vertex") {
      read_vertices = true;
      if (header.format == "binary_little_endian") {
        cloud = read_binary_vertices(input, element, bounds);
      } else {
        cloud = read_ascii_vertices(input, element, bounds);
      }
      continue;
    }

    if (header.format == "binary_little_endian") {
      skip_binary_element(input, element);
    } else {
      std::string ignored;
      for (std::uint64_t i = 0; i < element.count; ++i) {
        if (!std::getline(input, ignored)) {
          throw std::runtime_error("unexpected EOF while skipping ASCII PLY element");
        }
      }
    }
  }

  if (!read_vertices || !cloud) {
    throw std::runtime_error("PLY file has no vertex element");
  }
  if (cloud->empty()) {
    throw std::runtime_error("PLY vertex cloud is empty after filtering invalid points");
  }
  return cloud;
}

std::string get_arg_value(
  const std::string & arg,
  int & index,
  const int argc,
  char ** argv,
  const std::string & name)
{
  const auto prefix = name + "=";
  if (arg.rfind(prefix, 0) == 0) {
    return arg.substr(prefix.size());
  }
  if (index + 1 >= argc) {
    throw std::runtime_error("missing value for " + name);
  }
  ++index;
  return argv[index];
}

double parse_double_arg(const std::string & value, const std::string & name)
{
  try {
    std::size_t parsed = 0;
    const double result = std::stod(value, &parsed);
    if (parsed != value.size()) {
      throw std::runtime_error("");
    }
    return result;
  } catch (const std::exception &) {
    throw std::runtime_error("invalid numeric value for " + name + ": " + value);
  }
}

void print_usage(std::ostream & output)
{
  output << R"(Usage:
  ros2 run ply_map_converter ply_map_converter --input <map.ply> --map-name <name> [options]

Options:
  --input <ply>                         Input PLY file
  --map-name <name>                     Output map name
  --bringup-maps-dir <dir>              Output maps root, default: src/bringup/maps
  --resolution <m>                      2D map resolution, default: 0.05
  --padding <m>                         XY map padding, default: 0.5
  --pcd-voxel-size <m>                  PCD voxel downsample size, default: 0.05; 0 disables
  --free-min-z <m>                      Free floor band min z, default: -0.05
  --free-max-z <m>                      Free floor band max z, default: 0.08
  --occupied-min-z <m>                  Occupied band min z, default: 0.15
  --occupied-max-z <m>                  Occupied band max z, default: 1.70
  --overwrite                           Replace existing output files
  -h, --help                            Show this help
)";
}

Options parse_options(const int argc, char ** argv)
{
  Options options;
  for (int i = 1; i < argc; ++i) {
    const std::string arg(argv[i]);
    if (arg == "-h" || arg == "--help") {
      print_usage(std::cout);
      std::exit(0);
    } else if (arg == "--overwrite") {
      options.overwrite = true;
    } else if (arg == "--input" || arg.rfind("--input=", 0) == 0) {
      options.input = get_arg_value(arg, i, argc, argv, "--input");
    } else if (arg == "--map-name" || arg.rfind("--map-name=", 0) == 0) {
      options.map_name = get_arg_value(arg, i, argc, argv, "--map-name");
    } else if (arg == "--bringup-maps-dir" || arg.rfind("--bringup-maps-dir=", 0) == 0) {
      options.bringup_maps_dir = get_arg_value(arg, i, argc, argv, "--bringup-maps-dir");
    } else if (arg == "--resolution" || arg.rfind("--resolution=", 0) == 0) {
      options.resolution = parse_double_arg(
        get_arg_value(arg, i, argc, argv, "--resolution"), "--resolution");
    } else if (arg == "--padding" || arg.rfind("--padding=", 0) == 0) {
      options.padding = parse_double_arg(
        get_arg_value(arg, i, argc, argv, "--padding"), "--padding");
    } else if (arg == "--pcd-voxel-size" || arg.rfind("--pcd-voxel-size=", 0) == 0) {
      options.pcd_voxel_size = parse_double_arg(
        get_arg_value(arg, i, argc, argv, "--pcd-voxel-size"), "--pcd-voxel-size");
    } else if (arg == "--free-min-z" || arg.rfind("--free-min-z=", 0) == 0) {
      options.free_min_z = parse_double_arg(
        get_arg_value(arg, i, argc, argv, "--free-min-z"), "--free-min-z");
    } else if (arg == "--free-max-z" || arg.rfind("--free-max-z=", 0) == 0) {
      options.free_max_z = parse_double_arg(
        get_arg_value(arg, i, argc, argv, "--free-max-z"), "--free-max-z");
    } else if (arg == "--occupied-min-z" || arg.rfind("--occupied-min-z=", 0) == 0) {
      options.occupied_min_z = parse_double_arg(
        get_arg_value(arg, i, argc, argv, "--occupied-min-z"), "--occupied-min-z");
    } else if (arg == "--occupied-max-z" || arg.rfind("--occupied-max-z=", 0) == 0) {
      options.occupied_max_z = parse_double_arg(
        get_arg_value(arg, i, argc, argv, "--occupied-max-z"), "--occupied-max-z");
    } else {
      throw std::runtime_error("unknown argument: " + arg);
    }
  }

  if (options.input.empty()) {
    throw std::runtime_error("--input is required");
  }
  if (options.map_name.empty()) {
    throw std::runtime_error("--map-name is required");
  }
  if (options.resolution <= 0.0) {
    throw std::runtime_error("--resolution must be positive");
  }
  if (options.padding < 0.0) {
    throw std::runtime_error("--padding must be non-negative");
  }
  if (options.pcd_voxel_size < 0.0) {
    throw std::runtime_error("--pcd-voxel-size must be non-negative");
  }
  if (options.free_min_z > options.free_max_z) {
    throw std::runtime_error("--free-min-z must be <= --free-max-z");
  }
  if (options.occupied_min_z > options.occupied_max_z) {
    throw std::runtime_error("--occupied-min-z must be <= --occupied-max-z");
  }
  return options;
}

OutputPaths make_output_paths(const Options & options)
{
  OutputPaths paths;
  paths.root = options.bringup_maps_dir / options.map_name;
  paths.pcd = paths.root / "3D" / "map.pcd";
  paths.pgm = paths.root / "2D" / (options.map_name + "_2d.pgm");
  paths.yaml = paths.root / "2D" / (options.map_name + "_2d.yaml");
  return paths;
}

void check_output_paths(const OutputPaths & paths, const bool overwrite)
{
  if (overwrite) {
    return;
  }

  std::vector<fs::path> existing;
  for (const auto & path : {paths.pcd, paths.pgm, paths.yaml}) {
    if (fs::exists(path)) {
      existing.push_back(path);
    }
  }
  if (!existing.empty()) {
    std::ostringstream message;
    message << "output file already exists; pass --overwrite to replace:";
    for (const auto & path : existing) {
      message << "\n  " << path.string();
    }
    throw std::runtime_error(message.str());
  }
}

pcl::PointCloud<pcl::PointXYZI>::Ptr downsample_for_pcd(
  const pcl::PointCloud<pcl::PointXYZI>::ConstPtr & cloud,
  const double voxel_size)
{
  if (voxel_size <= 0.0) {
    auto copy = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
    *copy = *cloud;
    return copy;
  }

  pcl::VoxelGrid<pcl::PointXYZI> filter;
  filter.setInputCloud(cloud);
  const auto leaf = static_cast<float>(voxel_size);
  filter.setLeafSize(leaf, leaf, leaf);

  auto output = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
  filter.filter(*output);
  output->width = static_cast<std::uint32_t>(output->points.size());
  output->height = 1;
  output->is_dense = false;
  return output;
}

std::size_t grid_index(const std::size_t x, const std::size_t y, const std::size_t width)
{
  return y * width + x;
}

void write_pgm(
  const fs::path & path,
  const std::vector<std::uint8_t> & grid,
  const std::size_t width,
  const std::size_t height)
{
  std::ofstream output(path, std::ios::binary);
  if (!output) {
    throw std::runtime_error("failed to open PGM output: " + path.string());
  }

  output << "P5\n";
  output << "# CREATOR: ply_map_converter\n";
  output << width << " " << height << "\n";
  output << "255\n";

  for (std::size_t row = 0; row < height; ++row) {
    const std::size_t y = height - 1 - row;
    output.write(
      reinterpret_cast<const char *>(&grid[grid_index(0, y, width)]),
      static_cast<std::streamsize>(width));
  }
  if (!output) {
    throw std::runtime_error("failed while writing PGM output: " + path.string());
  }
}

void write_yaml(
  const fs::path & path,
  const fs::path & image_name,
  const double resolution,
  const double origin_x,
  const double origin_y)
{
  std::ofstream output(path);
  if (!output) {
    throw std::runtime_error("failed to open YAML output: " + path.string());
  }

  output << std::setprecision(10);
  output << "image: " << image_name.filename().string() << "\n";
  output << "mode: trinary\n";
  output << "resolution: " << resolution << "\n";
  output << "origin: [" << origin_x << ", " << origin_y << ", 0]\n";
  output << "negate: 0\n";
  output << "occupied_thresh: 0.65\n";
  output << "free_thresh: 0.25\n";
}

void write_2d_map(
  const pcl::PointCloud<pcl::PointXYZI>::ConstPtr & cloud,
  const Bounds & bounds,
  const Options & options,
  const OutputPaths & paths)
{
  const double origin_x = bounds.min_x - options.padding;
  const double origin_y = bounds.min_y - options.padding;
  const double max_x = bounds.max_x + options.padding;
  const double max_y = bounds.max_y + options.padding;
  const auto width =
    static_cast<std::size_t>(std::ceil((max_x - origin_x) / options.resolution)) + 1;
  const auto height =
    static_cast<std::size_t>(std::ceil((max_y - origin_y) / options.resolution)) + 1;

  if (width == 0 || height == 0 || width > 1000000 || height > 1000000) {
    throw std::runtime_error("computed 2D map dimensions are invalid");
  }
  if (width > std::numeric_limits<std::size_t>::max() / height) {
    throw std::runtime_error("computed 2D map is too large");
  }

  std::vector<std::uint8_t> grid(width * height, kPgmUnknown);
  std::size_t free_cells = 0;
  std::size_t occupied_cells = 0;

  for (const auto & point : cloud->points) {
    const auto x = static_cast<std::int64_t>(
      std::floor((static_cast<double>(point.x) - origin_x) / options.resolution));
    const auto y = static_cast<std::int64_t>(
      std::floor((static_cast<double>(point.y) - origin_y) / options.resolution));
    if (x < 0 || y < 0 || x >= static_cast<std::int64_t>(width) ||
      y >= static_cast<std::int64_t>(height))
    {
      continue;
    }

    const auto idx = grid_index(static_cast<std::size_t>(x), static_cast<std::size_t>(y), width);
    const auto z = static_cast<double>(point.z);
    if (z >= options.free_min_z && z <= options.free_max_z && grid[idx] == kPgmUnknown) {
      grid[idx] = kPgmFree;
      ++free_cells;
    }
    if (z >= options.occupied_min_z && z <= options.occupied_max_z &&
      grid[idx] != kPgmOccupied)
    {
      if (grid[idx] == kPgmFree && free_cells > 0) {
        --free_cells;
      }
      grid[idx] = kPgmOccupied;
      ++occupied_cells;
    }
  }

  write_pgm(paths.pgm, grid, width, height);
  write_yaml(paths.yaml, paths.pgm.filename(), options.resolution, origin_x, origin_y);

  std::cout << "2D map: " << width << "x" << height
            << " cells, free=" << free_cells
            << " occupied=" << occupied_cells
            << " unknown=" << (grid.size() - free_cells - occupied_cells) << "\n";
}

void ensure_output_dirs(const OutputPaths & paths)
{
  fs::create_directories(paths.pcd.parent_path());
  fs::create_directories(paths.pgm.parent_path());
}

int run(const int argc, char ** argv)
{
  const auto options = parse_options(argc, argv);
  if (!fs::exists(options.input)) {
    throw std::runtime_error("input PLY does not exist: " + options.input.string());
  }

  const auto paths = make_output_paths(options);
  check_output_paths(paths, options.overwrite);

  Bounds bounds;
  std::cout << "Reading PLY: " << options.input.string() << "\n";
  const auto raw_cloud = load_ply(options.input, bounds);
  std::cout << std::setprecision(4);
  std::cout << "Loaded points: " << raw_cloud->size()
            << " bounds x=[" << bounds.min_x << ", " << bounds.max_x << "]"
            << " y=[" << bounds.min_y << ", " << bounds.max_y << "]"
            << " z=[" << bounds.min_z << ", " << bounds.max_z << "]\n";

  const auto pcd_cloud = downsample_for_pcd(raw_cloud, options.pcd_voxel_size);
  if (pcd_cloud->empty()) {
    throw std::runtime_error("PCD cloud is empty after downsampling");
  }

  ensure_output_dirs(paths);

  if (pcl::io::savePCDFileBinary(paths.pcd.string(), *pcd_cloud) != 0) {
    throw std::runtime_error("failed to write PCD output: " + paths.pcd.string());
  }
  std::cout << "PCD map: " << paths.pcd.string()
            << " points=" << pcd_cloud->size()
            << " voxel_size=" << options.pcd_voxel_size << "\n";

  write_2d_map(raw_cloud, bounds, options, paths);
  std::cout << "PGM map: " << paths.pgm.string() << "\n";
  std::cout << "YAML map: " << paths.yaml.string() << "\n";
  return 0;
}

}  // namespace

int main(int argc, char ** argv)
{
  try {
    return run(argc, argv);
  } catch (const std::exception & error) {
    std::cerr << "ply_map_converter: " << error.what() << "\n\n";
    print_usage(std::cerr);
    return 1;
  }
}
