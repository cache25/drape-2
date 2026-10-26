#pragma once

#include <QElapsedTimer>
#include <QObject>
#include <QTimer>

#include <cstdint>
#include <filesystem>

namespace drape::app {

class MainWindow;

// `Drape --selftest <out.png>` (Phase B spec 7.2): drapes the tee on the male at Standard on the GPU and prints, one
// per line: launch_ms, fps_asleep, tee_coverage, then "settled=<0|1> backend=<name> seconds=<s>". Resizes the window
// to 640x480 at 1 s and back to 1280x800 at 2 s, waits up to 180 s for the drape to finish, measures the frame rate
// over 2 s with the sim asleep, saves an offscreen 1280x800 render to the PNG, and exits 0 only if the tee settled.
class SelfTest : public QObject {
  Q_OBJECT

 public:
  SelfTest(MainWindow& window, std::filesystem::path png, const QElapsedTimer& sinceLaunch);

 private:
  void poll();
  void finish();

  MainWindow& window_;
  std::filesystem::path png_;
  QElapsedTimer sinceLaunch_;
  QElapsedTimer sinceStart_;
  QTimer pollTimer_;
  bool resizedSmall_ = false, resizedBack_ = false, measuring_ = false;
  std::uint64_t framesAtMeasureStart_ = 0;
  qint64 measureStartMs_ = 0;
};

}  // namespace drape::app
