#include "self_test.hpp"

#include <QApplication>
#include <QTimer>

#include <cstdio>
#include <exception>
#include <string>

#include "drape/render/image_metrics.hpp"
#include "main_window.hpp"
#include "viewport_widget.hpp"

namespace drape::app {

SelfTest::SelfTest(MainWindow& window, std::filesystem::path png, const QElapsedTimer& sinceLaunch)
    : window_(window), png_(std::move(png)), sinceLaunch_(sinceLaunch) {
  sinceStart_.start();
  connect(&window_.viewport(), &ViewportWidget::firstFramePresented, this, [this] {
    std::printf("launch_ms=%lld\n", static_cast<long long>(sinceLaunch_.elapsed()));
    std::fflush(stdout);
  });
  connect(&window_.viewport(), &ViewportWidget::rendererFailed, this, [](const QString& why) {
    std::printf("renderer failed: %s\n", why.toStdString().c_str());
    std::fflush(stdout);
    QTimer::singleShot(0, qApp, [] { QApplication::exit(1); });  // may fire before app.exec() runs
  });
  connect(&pollTimer_, &QTimer::timeout, this, [this] { poll(); });
  pollTimer_.start(50);
}

void SelfTest::poll() {
  const qint64 t = sinceStart_.elapsed();
  if (!resizedSmall_ && t >= 1000) {
    resizedSmall_ = true;
    window_.resize(640, 480);
  }
  if (!resizedBack_ && t >= 2000) {
    resizedBack_ = true;
    window_.resize(1280, 800);
  }
  const session::SimStatus s = window_.controller().status();
  const bool finished = s.state == sim::DrapeState::Settled || s.state == sim::DrapeState::Failed ||
                        s.state == sim::DrapeState::TimedOut;
  if (!measuring_ && resizedBack_ && ((finished && !s.running) || t >= 180000)) {
    measuring_ = true;
    framesAtMeasureStart_ = window_.viewport().framesPresented();
    measureStartMs_ = t;
  }
  if (measuring_ && t - measureStartMs_ >= 2000) finish();
}

void SelfTest::finish() {
  pollTimer_.stop();
  const double seconds = (sinceStart_.elapsed() - measureStartMs_) / 1000.0;
  const double fps = (window_.viewport().framesPresented() - framesAtMeasureStart_) / seconds;
  std::printf("fps_asleep=%.1f\n", fps);
  const session::SimStatus s = window_.controller().status();
  int code = s.state == sim::DrapeState::Settled ? 0 : 1;
  try {
    render::Renderer* r = window_.viewport().renderer();
    if (!r) throw std::runtime_error("no renderer");
    const render::RgbaImage img = r->renderOffscreen(1280, 800);
    render::writePng(png_, img.pixels, img.width, img.height);
    std::printf("tee_coverage=%.3f\n", render::coverage(img.pixels, img.width, img.height, render::isBlueTint));
  } catch (const std::exception& e) {
    std::printf("screenshot failed: %s\n", e.what());
    code = 1;
  }
  std::printf("settled=%d backend=%s seconds=%.2f particles=%zu message=%s\n",
              s.state == sim::DrapeState::Settled ? 1 : 0, std::string(sim::backendName(s.backend)).c_str(),
              s.simulatedSeconds, s.particles, s.message.c_str());
  std::fflush(stdout);
  QApplication::exit(code);
}

}  // namespace drape::app
