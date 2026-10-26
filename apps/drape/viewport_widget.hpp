#pragma once

#include <QElapsedTimer>
#include <QPoint>
#include <QTimer>
#include <QWidget>

#include <cstdint>
#include <memory>
#include <optional>

#include "drape/avatar/mannequin.hpp"
#include "drape/render/renderer.hpp"
#include "drape/session/sim_controller.hpp"

namespace drape::app {

// The 3D viewport (Phase B spec 4.2): a native NSView with a CAMetalLayer that Filament draws into. It pulls the
// latest published frame from the SimController every tick, re-uploads the meshes when the scene revision changes
// (framing the mannequin when the body changes) and renders at the display rate whether or not the sim is running.
class ViewportWidget : public QWidget {
  Q_OBJECT

 public:
  ViewportWidget(session::SimController& controller, render::RendererAssets assets, QWidget* parent = nullptr);
  ~ViewportWidget() override;

  render::Renderer* renderer() { return renderer_.get(); }
  std::uint64_t framesPresented() const { return framesPresented_; }
  QPaintEngine* paintEngine() const override { return nullptr; }

 signals:
  void rendererFailed(const QString& why);
  void firstFramePresented();

 protected:
  void showEvent(QShowEvent* e) override;
  void resizeEvent(QResizeEvent* e) override;
  void paintEvent(QPaintEvent*) override {}
  void mousePressEvent(QMouseEvent* e) override;
  void mouseMoveEvent(QMouseEvent* e) override;
  void wheelEvent(QWheelEvent* e) override;
  bool event(QEvent* e) override;

 private:
  void tick();
  void updateDrawableSize();

  session::SimController& controller_;
  render::RendererAssets assets_;
  std::unique_ptr<render::Renderer> renderer_;
  void* layer_ = nullptr;
  QTimer timer_;
  session::PublishedFrame frame_;
  std::uint64_t revision_ = 0;
  std::optional<avatar::BodyType> framedBody_;
  std::uint64_t framesPresented_ = 0;
  QPoint lastMouse_;
};

}  // namespace drape::app
