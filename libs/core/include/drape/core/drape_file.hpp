#pragma once

#include <cstdint>
#include <filesystem>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "drape/core/project.hpp"

namespace drape {

inline constexpr int kDrapeFormatVersion = 1;

enum class FileErrorCode { Io, NotAZip, MissingEntry, InvalidJson, SchemaMismatch, NewerFormat };

class FileError : public std::runtime_error {
 public:
  FileError(FileErrorCode code, const std::string& message) : std::runtime_error(message), code_(code) {}
  FileErrorCode code() const { return code_; }

 private:
  FileErrorCode code_;
};

// Settled particle positions for one garment, valid only while `key` matches garmentCacheKey().
struct CacheEntry {
  std::uint64_t key = 0;
  std::vector<Vec3f> positions;
  bool operator==(const CacheEntry& o) const {
    if (key != o.key || positions.size() != o.positions.size()) return false;
    for (std::size_t i = 0; i < positions.size(); ++i) {
      if (positions[i] != o.positions[i]) return false;
    }
    return true;
  }
};

struct DrapeFile {
  Project project;
  std::map<Id, CacheEntry> cache;
  std::vector<std::uint8_t> thumbnailPng;
  bool operator==(const DrapeFile&) const = default;
};

void saveDrapeFile(const std::filesystem::path& path, const DrapeFile& file);  // throws FileError(Io)
DrapeFile loadDrapeFile(const std::filesystem::path& path);                    // throws FileError
nlohmann::json migrateProjectJson(nlohmann::json j, int fromVersion);          // v1: identity; >1 throws NewerFormat
std::uint64_t garmentCacheKey(const Project& project, const GarmentInstance& garment);

}  // namespace drape
