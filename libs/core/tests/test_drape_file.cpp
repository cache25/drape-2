#include <gtest/gtest.h>
#include <miniz.h>

#include <filesystem>
#include <fstream>
#include <map>
#include <string>

#include "drape/core/drape_file.hpp"
#include "drape/core/hash.hpp"
#include "drape/core/serialize.hpp"
#include "sample_project.hpp"

namespace fs = std::filesystem;
using drape::DrapeFile;
using drape::FileError;
using drape::FileErrorCode;

namespace {

fs::path freshDir(const std::string& name) {
  fs::path d = fs::temp_directory_path() / "drape_file_tests" / name;
  fs::remove_all(d);
  fs::create_directories(d);
  return d;
}

void writeZip(const fs::path& path, const std::map<std::string, std::string>& entries) {
  mz_zip_archive zip{};
  ASSERT_TRUE(mz_zip_writer_init_file(&zip, path.string().c_str(), 0));
  for (const auto& [name, data] : entries) {
    ASSERT_TRUE(mz_zip_writer_add_mem(&zip, name.c_str(), data.data(), data.size(), MZ_DEFAULT_COMPRESSION));
  }
  ASSERT_TRUE(mz_zip_writer_finalize_archive(&zip));
  mz_zip_writer_end(&zip);
}

std::string readZipEntry(const fs::path& path, const std::string& name) {
  mz_zip_archive zip{};
  if (!mz_zip_reader_init_file(&zip, path.string().c_str(), 0)) return {};
  size_t size = 0;
  void* p = mz_zip_reader_extract_file_to_heap(&zip, name.c_str(), &size, 0);
  std::string out = p ? std::string(static_cast<char*>(p), size) : std::string();
  mz_free(p);
  mz_zip_reader_end(&zip);
  return out;
}

DrapeFile sampleFile() {
  DrapeFile f;
  f.project = drape::testing::makeSampleProject();
  f.cache["g1"] = drape::CacheEntry{42, {{0, 1, 2}, {3, 4, 5}, {6, 7, 8.5f}}};
  f.thumbnailPng = {0x89, 'P', 'N', 'G'};
  return f;
}

std::string validManifest(int version) {
  return "{\"formatVersion\":" + std::to_string(version) + ",\"appVersion\":\"0.1.0-dev\",\"created\":\"\",\"modified\":\"\"}";
}

std::string validProjectJson() { return nlohmann::json(drape::testing::makeSampleProject()).dump(); }

FileErrorCode loadCode(const fs::path& p, std::string* what = nullptr) {
  try {
    drape::loadDrapeFile(p);
  } catch (const FileError& e) {
    if (what) *what = e.what();
    return e.code();
  }
  ADD_FAILURE() << "expected FileError";
  return FileErrorCode::Io;
}

}  // namespace

TEST(DrapeFile, RoundTrip) {
  auto p = freshDir("roundtrip") / "a.drape";
  DrapeFile f = sampleFile();
  drape::saveDrapeFile(p, f);
  EXPECT_EQ(drape::loadDrapeFile(p), f);
}

TEST(DrapeFile, ManifestContents) {
  auto p = freshDir("manifest") / "a.drape";
  drape::saveDrapeFile(p, sampleFile());
  auto m = nlohmann::json::parse(readZipEntry(p, "manifest.json"));
  EXPECT_EQ(m.at("formatVersion"), 1);
  EXPECT_EQ(m.at("appVersion"), "0.1.0-dev");
  EXPECT_EQ(m.at("modified"), "2026-10-01T01:00:00Z");
}

TEST(DrapeFile, NewerFormatRefused) {
  auto p = freshDir("newer") / "a.drape";
  writeZip(p, {{"manifest.json", validManifest(2)}, {"project.json", validProjectJson()}});
  std::string what;
  EXPECT_EQ(loadCode(p, &what), FileErrorCode::NewerFormat);
  EXPECT_NE(what.find("newer version of Drape"), std::string::npos) << what;
}

TEST(DrapeFile, NotAZip) {
  auto p = freshDir("notzip") / "a.drape";
  std::ofstream(p) << "hello, I am not a zip";
  EXPECT_EQ(loadCode(p), FileErrorCode::NotAZip);
}

TEST(DrapeFile, MissingProjectJson) {
  auto p = freshDir("missing") / "a.drape";
  writeZip(p, {{"manifest.json", validManifest(1)}});
  std::string what;
  EXPECT_EQ(loadCode(p, &what), FileErrorCode::MissingEntry);
  EXPECT_NE(what.find("project.json"), std::string::npos) << what;
}

TEST(DrapeFile, InvalidJson) {
  auto p = freshDir("invalid") / "a.drape";
  writeZip(p, {{"manifest.json", validManifest(1)}, {"project.json", "{"}});
  EXPECT_EQ(loadCode(p), FileErrorCode::InvalidJson);
}

TEST(DrapeFile, NullFieldIsSchemaMismatch) {
  auto p = freshDir("nullfield") / "a.drape";
  auto j = nlohmann::json(drape::testing::makeSampleProject());
  j["units"] = nullptr;
  writeZip(p, {{"manifest.json", validManifest(1)}, {"project.json", j.dump()}});
  std::string what;
  EXPECT_EQ(loadCode(p, &what), FileErrorCode::SchemaMismatch);
  EXPECT_NE(what.find("project.json"), std::string::npos) << what;
  EXPECT_NE(what.find("/units"), std::string::npos) << what;
}

TEST(DrapeFile, TruncatedFileIsNotAZip) {
  auto p = freshDir("truncated") / "a.drape";
  drape::saveDrapeFile(p, sampleFile());
  fs::resize_file(p, fs::file_size(p) / 2);
  EXPECT_EQ(loadCode(p), FileErrorCode::NotAZip);
}

TEST(DrapeFile, CorruptCacheEntryIgnored) {
  auto p = freshDir("corruptcache") / "a.drape";
  writeZip(p, {{"manifest.json", validManifest(1)},
               {"project.json", validProjectJson()},
               {"cache/g1.bin", "XXXX garbage"}});
  DrapeFile f = drape::loadDrapeFile(p);
  EXPECT_EQ(f.cache.count("g1"), 0u);
  EXPECT_EQ(f.project, drape::testing::makeSampleProject());
}

TEST(DrapeFile, CacheKeyTracksInputs) {
  auto project = drape::testing::makeSampleProject();
  const auto& g = project.garments[0];
  const auto base = drape::garmentCacheKey(project, g);
  EXPECT_EQ(drape::garmentCacheKey(project, g), base);

  auto changedPattern = g;
  changedPattern.pattern.seams[0].ease = 0.002;
  EXPECT_NE(drape::garmentCacheKey(project, changedPattern), base);

  auto changedQuality = g;
  changedQuality.quality = drape::SimQuality::Draft;
  EXPECT_NE(drape::garmentCacheKey(project, changedQuality), base);

  auto changedAvatar = project;
  changedAvatar.avatar.targets[drape::Measurement::Waist] = 0.90;
  EXPECT_NE(drape::garmentCacheKey(changedAvatar, g), base);

  auto changedFabric = project;
  changedFabric.customFabrics[0].physical.bendWarp = 1e-6;
  EXPECT_NE(drape::garmentCacheKey(changedFabric, g), base);
}

TEST(DrapeFile, SaveIsAtomic) {
  auto dir = freshDir("atomic");
  auto p = dir / "a.drape";
  DrapeFile f = sampleFile();
  drape::saveDrapeFile(p, f);
  f.project.meta.name = "Second";
  drape::saveDrapeFile(p, f);
  EXPECT_EQ(drape::loadDrapeFile(p).project.meta.name, "Second");
  for (const auto& entry : fs::directory_iterator(dir)) {
    EXPECT_NE(entry.path().extension(), ".tmp") << entry.path();
  }
}

TEST(DrapeFile, MigrationIdentityAndNewer) {
  nlohmann::json j = drape::testing::makeSampleProject();
  EXPECT_EQ(drape::migrateProjectJson(j, 1), j);
  try {
    drape::migrateProjectJson(j, 2);
    ADD_FAILURE() << "expected FileError";
  } catch (const FileError& e) {
    EXPECT_EQ(e.code(), FileErrorCode::NewerFormat);
  }
}

TEST(Hash, Fnv1a64KnownValues) {
  EXPECT_EQ(drape::fnv1a64(""), 0xcbf29ce484222325ull);
  EXPECT_EQ(drape::fnv1a64("a"), 0xaf63dc4c8601ec8cull);
}
