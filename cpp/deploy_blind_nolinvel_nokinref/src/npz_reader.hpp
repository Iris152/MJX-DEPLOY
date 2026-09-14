#pragma once
/// 文件：npz_reader.hpp
/// 最小化 .npz / .npy 读取器，支持 numpy v1.0 和 v2.0 格式。
/// 用于替代只支持 v1.0 且遇到 v2.0 头部会崩溃的 cnpy。
///
/// 仅支持读取、小端 float32/float64，以及 C 顺序数组；策略部署只需要这些能力。

#include <Eigen/Core>

#include <cstdint>
#include <cstring>
#include <fstream>
#include <regex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

// .npz 本质上是 zip 文件，这里用 zlib 解压。
#include <zlib.h>

namespace jave {
namespace npz {

// 从 .npy 中读出的单个数组。

struct NpyArray {
  std::vector<char> data;
  std::vector<size_t> shape;
  size_t word_size = 0; // 4 表示 float32，8 表示 float64。
  bool is_float = true;

  size_t num_elements() const {
    size_t n = 1;
    for (auto s : shape)
      n *= s;
    return n;
  }

  /// 以 double 形式读取元素，透明处理 f4/f8。
  double as_double(size_t i) const {
    if (word_size == 8)
      return reinterpret_cast<const double *>(data.data())[i];
    else
      return static_cast<double>(
          reinterpret_cast<const float *>(data.data())[i]);
  }

  /// 加载为 Eigen VectorXd。
  Eigen::VectorXd to_vector() const {
    const size_t n = num_elements();
    Eigen::VectorXd v(n);
    for (size_t i = 0; i < n; ++i)
      v(static_cast<int>(i)) = as_double(i);
    return v;
  }

  /// 加载为 Eigen MatrixXd，将 numpy 行主序转换为 Eigen 列主序。
  Eigen::MatrixXd to_matrix() const {
    if (shape.size() != 2)
      throw std::runtime_error("to_matrix: array is not 2-D");
    const int rows = static_cast<int>(shape[0]);
    const int cols = static_cast<int>(shape[1]);
    Eigen::MatrixXd m(rows, cols);
    for (int r = 0; r < rows; ++r)
      for (int c = 0; c < cols; ++c)
        m(r, c) = as_double(r * cols + c);
    return m;
  }

  /// 加载标量，支持 0 维或单元素数组。
  double to_scalar() const { return as_double(0); }
};

// 解析 .npy 二进制块，支持 v1.0 或 v2.0。

inline NpyArray parse_npy(const char *buf, size_t len) {
  // 魔数：\x93NUMPY。
  if (len < 10 || buf[0] != '\x93' || std::memcmp(buf + 1, "NUMPY", 5) != 0)
    throw std::runtime_error("parse_npy: not a valid .npy buffer");

  uint8_t major = static_cast<uint8_t>(buf[6]);
  // 小版本号当前不参与分支判断。

  uint32_t header_len = 0;
  size_t header_offset = 0;

  if (major == 1) {
    // v1.0：偏移 8 处为 2 字节小端头部长度。
    header_len = static_cast<uint16_t>(static_cast<uint8_t>(buf[8]) |
                                       (static_cast<uint8_t>(buf[9]) << 8));
    header_offset = 10;
  } else if (major >= 2) {
    // v2.0+：偏移 8 处为 4 字节小端头部长度。
    header_len = static_cast<uint32_t>(static_cast<uint8_t>(buf[8]) |
                                       (static_cast<uint8_t>(buf[9]) << 8) |
                                       (static_cast<uint8_t>(buf[10]) << 16) |
                                       (static_cast<uint8_t>(buf[11]) << 24));
    header_offset = 12;
  } else {
    throw std::runtime_error("parse_npy: unsupported npy major version");
  }

  if (header_offset + header_len > len)
    throw std::runtime_error("parse_npy: header extends past buffer");

  std::string header(buf + header_offset, header_len);
  const char *data_start = buf + header_offset + header_len;
  size_t data_len = len - header_offset - header_len;

  // 解析头部字典，例如 {'descr': '<f8', 'fortran_order': False, 'shape':
  // (49,), }。
  NpyArray arr;

  // 数据类型。
  std::regex descr_re("'descr'\\s*:\\s*'([^']*)'");
  std::smatch m;
  if (!std::regex_search(header, m, descr_re))
    throw std::runtime_error("parse_npy: no descr in header");
  std::string descr = m[1].str();

  if (descr == "<f8" || descr == "=f8" || descr == "f8")
    arr.word_size = 8;
  else if (descr == "<f4" || descr == "=f4" || descr == "f4")
    arr.word_size = 4;
  else if (descr == "<i8" || descr == "<i4" || descr == "<u8" || descr == "<u4")
    // 整数标量，例如 n_hidden。
    arr.word_size = (descr.back() == '8') ? 8 : 4;
  else
    throw std::runtime_error("parse_npy: unsupported dtype: " + descr);

  arr.is_float = (descr.find('f') != std::string::npos);

  // 数组形状。
  std::regex shape_re("'shape'\\s*:\\s*\\(([^)]*)\\)");
  if (!std::regex_search(header, m, shape_re))
    throw std::runtime_error("parse_npy: no shape in header");
  std::string shape_str = m[1].str();
  // 解析逗号分隔的整数；0 维可为空，1 维常见形式为 "49,"。
  {
    std::istringstream ss(shape_str);
    std::string tok;
    while (std::getline(ss, tok, ',')) {
      // 去掉首尾空白。
      tok.erase(0, tok.find_first_not_of(" \t\n"));
      tok.erase(tok.find_last_not_of(" \t\n") + 1);
      if (!tok.empty())
        arr.shape.push_back(std::stoull(tok));
    }
  }

  // 复制数组数据。
  size_t expected = arr.num_elements() * arr.word_size;
  if (data_len < expected)
    throw std::runtime_error("parse_npy: data truncated (expected " +
                             std::to_string(expected) + " bytes, got " +
                             std::to_string(data_len) + ")");

  // 对整数标量，转换成 double 并存入 float64 缓冲区。
  if (!arr.is_float) {
    arr.data.resize(arr.num_elements() * 8);
    for (size_t i = 0; i < arr.num_elements(); ++i) {
      double val = 0;
      if (arr.word_size == 8) {
        int64_t iv;
        std::memcpy(&iv, data_start + i * 8, 8);
        val = static_cast<double>(iv);
      } else {
        int32_t iv;
        std::memcpy(&iv, data_start + i * 4, 4);
        val = static_cast<double>(iv);
      }
      std::memcpy(arr.data.data() + i * 8, &val, 8);
    }
    arr.word_size = 8;
    arr.is_float = true;
  } else {
    arr.data.assign(data_start, data_start + expected);
  }

  return arr;
}

// 加载 .npz，即多个 .npy 文件组成的 zip。

/// 通过 ZIP 中央目录读取 .npz，可处理 np.savez_compressed 生成的本地头部长度为 0 的情况。

using NpzFile = std::unordered_map<std::string, NpyArray>;

inline NpzFile load_npz(const std::string &path) {
  std::ifstream ifs(path, std::ios::binary);
  if (!ifs)
    throw std::runtime_error("load_npz: cannot open " + path);

  ifs.seekg(0, std::ios::end);
  size_t file_size = static_cast<size_t>(ifs.tellg());
  ifs.seekg(0, std::ios::beg);
  std::vector<char> fd(file_size);
  ifs.read(fd.data(), file_size);

  auto r16 = [&](size_t off) -> uint16_t {
    uint16_t v;
    std::memcpy(&v, fd.data() + off, 2);
    return v;
  };
  auto r32 = [&](size_t off) -> uint32_t {
    uint32_t v;
    std::memcpy(&v, fd.data() + off, 4);
    return v;
  };

  // 查找中央目录结束记录，位置在文件最后 22 字节或更靠前。
  // 签名：PK\x05\x06。
  size_t eocd_pos = file_size;
  {
    // ZIP 注释字段长度可能为 0..65535 字节，因此需要反向搜索。
    size_t search_start = (file_size > 65557) ? file_size - 65557 : 0;
    for (size_t i = file_size - 22; i >= search_start; --i) {
      if (fd[i] == 'P' && fd[i + 1] == 'K' && fd[i + 2] == 0x05 &&
          fd[i + 3] == 0x06) {
        eocd_pos = i;
        break;
      }
      if (i == 0)
        break;
    }
  }
  if (eocd_pos == file_size)
    throw std::runtime_error("load_npz: no EOCD record in " + path);

  uint32_t cd_size = r32(eocd_pos + 12);
  uint32_t cd_offset = r32(eocd_pos + 16);

  // 遍历中央目录条目。
  NpzFile npz;
  size_t cp = cd_offset;

  while (cp + 46 <= cd_offset + cd_size) {
    // 中央目录文件头：PK\x01\x02。
    if (fd[cp] != 'P' || fd[cp + 1] != 'K' || fd[cp + 2] != 0x01 ||
        fd[cp + 3] != 0x02)
      break;

    uint16_t method = r16(cp + 10);
    uint32_t comp_size = r32(cp + 20);
    uint32_t uncomp_size = r32(cp + 24);
    uint16_t name_len = r16(cp + 28);
    uint16_t extra_len = r16(cp + 30);
    uint16_t comment_len = r16(cp + 32);
    uint32_t local_off = r32(cp + 42);

    std::string name(fd.data() + cp + 46, name_len);

    // 移动到下一个中央目录条目。
    cp += 46 + name_len + extra_len + comment_len;

    // 从本地文件头定位实际数据。
    if (local_off + 30 > file_size)
      continue;
    uint16_t loc_name_len = r16(local_off + 26);
    uint16_t loc_extra_len = r16(local_off + 28);
    size_t data_pos = local_off + 30 + loc_name_len + loc_extra_len;

    if (data_pos + comp_size > file_size)
      throw std::runtime_error("load_npz: truncated entry: " + name);

    // 去掉 .npy 后缀作为键名。
    std::string key = name;
    if (key.size() > 4 && key.substr(key.size() - 4) == ".npy")
      key = key.substr(0, key.size() - 4);

    std::vector<char> npy_buf;

    if (method == 0) {
      // 未压缩存储。
      npy_buf.assign(fd.data() + data_pos, fd.data() + data_pos + comp_size);
    } else if (method == 8) {
      // 压缩数据。
      npy_buf.resize(uncomp_size);
      z_stream zs{};
      if (inflateInit2(&zs, -MAX_WBITS) != Z_OK)
        throw std::runtime_error("load_npz: inflateInit2 failed");
      zs.next_in = reinterpret_cast<Bytef *>(fd.data() + data_pos);
      zs.avail_in = comp_size;
      zs.next_out = reinterpret_cast<Bytef *>(npy_buf.data());
      zs.avail_out = uncomp_size;
      int ret = inflate(&zs, Z_FINISH);
      inflateEnd(&zs);
      if (ret != Z_STREAM_END)
        throw std::runtime_error("load_npz: inflate failed for " + name);
    } else {
      throw std::runtime_error("load_npz: unsupported compression method " +
                               std::to_string(method) + " for " + name);
    }

    npz[key] = parse_npy(npy_buf.data(), npy_buf.size());
  }

  return npz;
}

// 便捷辅助函数。

inline bool has_key(const NpzFile &npz, const std::string &key) {
  return npz.find(key) != npz.end();
}

} // 命名空间 npz
} // 命名空间 jave
