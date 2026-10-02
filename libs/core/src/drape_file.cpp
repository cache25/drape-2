#include "drape/core/drape_file.hpp"

#include <miniz.h>

#include <cstring>
#include <optional>
#include <set>

#include "drape/core/hash.hpp"
#include "drape/core/serialize.hpp"
#include "drape/core/version.hpp"

namespace drape {

namespace fs = std::filesystem;

namespace {

constexpr char kCacheMagic[4] = {'D', 'R', 'P', 'C'};
constexpr std::uint32_t kCacheVersion = 1;

void putU32(std::string& out, std::uint32_t v) {
  for (int i = 0; i < 4; ++i) out.push_back(static_cast<char>((v >> (8 * i)) & 0xffu));
}
void putU64(std::string& out, std::uint64_t v) {
  for (int i = 0; i < 8; ++i) out.push_back(static_cast<char>((v >> (8 * i)) & 0xffu));
}
std::uint64_t getLE(const std::string& in, std::size_t at, int bytes) {
  std::uint64_t v = 0;
  for (int i = 0; i < bytes; ++i) v |= static_cast<std::uint64_t>(static_cast<unsigned char>(in[at + i])) << (8 * i);
  return v;
}

std::string encodeCache(const CacheEntry& e) {
  std::string out(kCacheMagic, 4);
  putU32(out, kCacheVersion);
  putU64(out, e.key);
  putU32(out, static_cast<std::uint32_t>(e.positions.size()));
  for (const auto& p : e.positions) {
    for (int k = 0; k < 3; ++k) {
      std::uint32_t bits = 0;
      const float f = p[k];
      std::memcpy(&bits, &f, sizeof bits);
      putU32(out, bits);
    }
  }
  return out;
}

std::optional<CacheEntry> decodeCache(const std::string& in) {
  if (in.size() < 20 || std::memcmp(in.data(), kCacheMagic, 4) != 0) return std::nullopt;
  if (getLE(in, 4, 4) != kCacheVersion) return std::nullopt;
  CacheEntry e;
  e.key = getLE(in, 8, 8);
  const std::uint64_t count = getLE(in, 16, 4);
  if (in.size() != 20 + count * 12) return std::nullopt;
  e.positions.resize(count);
  for (std::uint64_t i = 0; i < count; ++i) {
    for (int k = 0; k < 3; ++k) {
      const auto bits = static_cast<std::uint32_t>(getLE(in, 20 + i * 12 + k * 4, 4));
      float f = 0;
      std::memcpy(&f, &bits, sizeof f);
      e.positions[i][k] = f;
    }
  }
  return e;
}

class ZipReader {
 public:
  explicit ZipReader(const fs::path& path) { ok_ = mz_zip_reader_init_file(&zip_, path.string().c_str(), 0); }
  ~ZipReader() {
    if (ok_) mz_zip_reader_end(&zip_);
  }
  ZipReader(const ZipReader&) = delete;
  ZipReader& operator=(const ZipReader&) = delete;

  bool ok() const { return ok_; }

  std::optional<std::string> read(const std::string& name) {
    size_t size = 0;
    void* p = mz_zip_reader_extract_file_to_heap(&zip_, name.c_str(), &size, 0);
    if (!p) return std::nullopt;
    std::string out(static_cast<char*>(p), size);
    mz_free(p);
    return out;
  }

  std::vector<std::string> names() {
    std::vector<std::string> out;
    const mz_uint n = mz_zip_reader_get_num_files(&zip_);
    for (mz_uint i = 0; i < n; ++i) {
      char buf[512];
      if (mz_zip_reader_get_filename(&zip_, i, buf, sizeof buf) > 0) out.emplace_back(buf);
    }
    return out;
  }

 private:
  mz_zip_archive zip_{};
  bool ok_ = false;
};

json parseEntry(const std::string& text, const std::string& entry, const fs::path& path) {
  try {
    return json::parse(text);
  } catch (const json::parse_error& e) {
    throw FileError(FileErrorCode::InvalidJson,
                    entry + " in " + path.string() + " is not valid JSON: " + std::string(e.what()));
  }
}

[[noreturn]] void schemaError(const std::string& entry, const fs::path& path, const std::exception& e) {
  throw FileError(FileErrorCode::SchemaMismatch,
                  entry + " in " + path.string() + " has unexpected content: " + std::string(e.what()));
}

}  // namespace

void saveDrapeFile(const fs::path& path, const DrapeFile& file) {
  const fs::path tmp = path.string() + ".tmp";
  auto fail = [&](const std::string& detail) {
    std::error_code ec;
    fs::remove(tmp, ec);
    throw FileError(FileErrorCode::Io, "could not write " + path.string() + ": " + detail);
  };

  const json manifest = {{"formatVersion", kDrapeFormatVersion},
                         {"appVersion", appVersion()},
                         {"created", file.project.meta.created},
                         {"modified", file.project.meta.modified}};
  std::vector<std::pair<std::string, std::string>> entries;
  entries.emplace_back("manifest.json", manifest.dump(2));
  entries.emplace_back("project.json", json(file.project).dump(2));
  for (const auto& [id, entry] : file.cache) entries.emplace_back("cache/" + id + ".bin", encodeCache(entry));
  if (!file.thumbnailPng.empty()) {
    entries.emplace_back("thumbnail.png", std::string(file.thumbnailPng.begin(), file.thumbnailPng.end()));
  }

  mz_zip_archive zip{};
  if (!mz_zip_writer_init_file(&zip, tmp.string().c_str(), 0)) fail("cannot create file");
  bool ok = true;
  for (const auto& [name, data] : entries) {
    ok = ok && mz_zip_writer_add_mem(&zip, name.c_str(), data.data(), data.size(), MZ_DEFAULT_COMPRESSION);
  }
  ok = ok && mz_zip_writer_finalize_archive(&zip);
  const char* err = mz_zip_get_error_string(mz_zip_get_last_error(&zip));
  mz_zip_writer_end(&zip);
  if (!ok) fail(err ? err : "zip error");

  std::error_code ec;
  fs::rename(tmp, path, ec);
  if (ec) fail(ec.message());
}

json migrateProjectJson(json j, int fromVersion) {
  if (fromVersion > kDrapeFormatVersion) {
    throw FileError(FileErrorCode::NewerFormat, "project was saved by a newer version of Drape (format " +
                                                    std::to_string(fromVersion) + "); this version reads format " +
                                                    std::to_string(kDrapeFormatVersion));
  }
  return j;  // format 1 is the only format so far
}

DrapeFile loadDrapeFile(const fs::path& path) {
  std::error_code ec;
  if (!fs::is_regular_file(path, ec)) {
    throw FileError(FileErrorCode::Io, "could not read " + path.string() + ": file not found");
  }
  ZipReader zip(path);
  if (!zip.ok()) throw FileError(FileErrorCode::NotAZip, path.string() + " is not a Drape project file");

  auto manifestText = zip.read("manifest.json");
  if (!manifestText) throw FileError(FileErrorCode::MissingEntry, path.string() + " is missing manifest.json");
  const json manifest = parseEntry(*manifestText, "manifest.json", path);
  int version = 0;
  try {
    version = manifest.at("formatVersion").get<int>();
  } catch (const json::exception& e) {
    schemaError("manifest.json", path, e);
  }
  if (version > kDrapeFormatVersion) {
    throw FileError(FileErrorCode::NewerFormat, path.string() + " was saved by a newer version of Drape (format " +
                                                    std::to_string(version) + "); this version reads format " +
                                                    std::to_string(kDrapeFormatVersion));
  }

  auto projectText = zip.read("project.json");
  if (!projectText) throw FileError(FileErrorCode::MissingEntry, path.string() + " is missing project.json");
  json projectJson = migrateProjectJson(parseEntry(*projectText, "project.json", path), version);

  DrapeFile out;
  try {
    out.project = projectJson.get<Project>();
  } catch (const json::exception& e) {
    schemaError("project.json", path, e);
  }

  for (const auto& name : zip.names()) {
    const std::string prefix = "cache/";
    const std::string suffix = ".bin";
    if (name.size() <= prefix.size() + suffix.size() || name.rfind(prefix, 0) != 0 ||
        name.compare(name.size() - suffix.size(), suffix.size(), suffix) != 0) {
      continue;
    }
    auto data = zip.read(name);
    if (!data) continue;
    if (auto entry = decodeCache(*data)) {
      out.cache[name.substr(prefix.size(), name.size() - prefix.size() - suffix.size())] = std::move(*entry);
    }
  }
  if (auto thumb = zip.read("thumbnail.png")) out.thumbnailPng.assign(thumb->begin(), thumb->end());
  return out;
}

std::uint64_t garmentCacheKey(const Project& project, const GarmentInstance& garment) {
  std::set<Id> referenced;
  for (const auto& [slot, id] : garment.fabricBySlot) referenced.insert(id);
  json fabrics = json::array();
  std::map<Id, const Fabric*> byId;
  for (const auto& f : project.customFabrics) byId[f.id] = &f;
  for (const auto& id : referenced) {
    if (auto it = byId.find(id); it != byId.end()) fabrics.push_back(*it->second);
  }
  const json key = {{"pattern", garment.pattern},
                    {"fabricBySlot", garment.fabricBySlot},
                    {"fabrics", fabrics},
                    {"quality", garment.quality},
                    {"avatar", project.avatar}};
  return fnv1a64(key.dump());
}

}  // namespace drape
