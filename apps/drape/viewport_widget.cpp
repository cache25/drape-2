#include "viewport_widget.hpp"

#include <QMouseEvent>
#include <QNativeGestureEvent>
#include <QResizeEvent>
#include <QWheelEvent>
#include <QWindow>

#include <exception>
#include <utility>

namespace drape::app {

ViewportWidget::ViewportWidget(session::SimController& controller, render::RendererAssets assets, QWidget* parent)
    : QWidget(parent), controller_(controller), assets_(std::move(assets)) {
  setAttribute(Qt::WA_NativeWindow);
  setAttribute(Qt::WA_PaintOnScreen);
  setAttribute(Qt::WA_NoSystemBackground);
  setMouseTracking(false);
  setMinimumSize(320, 240);
  timer_.setTimerType(Qt::PreciseTimer);
  connect(&timer_, &QTimer::timeout, this, [this] { tick(); });
}

ViewportWidget::~ViewportWidget() = default;

void ViewportWidget::showEvent(QShowEvent* e) {
  QWidget::showEvent(e);
  if (renderer_) return;
  try {
    renderer_ = std::make_unique<render::Renderer>(assets_);
    layer_ = render::makeMetalLayerForView(reinterpret_cast<void*>(winId()));
    renderer_->attachLayer(layer_);
    updateDrawableSize();
    if (QWindow* w = windowHandle()) connect(w, &QWindow::screenChanged, this, [this] { updateDrawableSize(); });
    timer_.start(8);  // the swap chain paces presentation to the display
  } catch (const std::exception& ex) {
    renderer_.reset();
    emit rendererFailed(QString::fromStdString(ex.what()));
  }
}

void ViewportWidget::resizeEvent(QResizeEvent* e) {
  QWidget::resizeEvent(e);
  updateDrawableSize();
}

void ViewportWidget::updateDrawableSize() {
  if (!renderer_ || !layer_) return;
  const double dpr = devicePixelRatioF();
  const auto w = static_cast<std::uint32_t>(width() * dpr + 0.5), h = static_cast<std::uint32_t>(height() * dpr + 0.5);
  render::resizeMetalLayer(layer_, dpr, w, h);
  renderer_->resize(w, h);
}

void ViewportWidget::tick() {
  if (!renderer_) return;
  if (controller_.exchange().latest(frame_, frame_.frame) && frame_.scene) {
    const session::TeeScene& scene = *frame_.scene;
    if (frame_.revision != revision_) {
      revision_ = frame_.revision;
      renderer_->setBody(scene.mannequin.mesh);
      std::vector<std::pair<std::uint32_t, std::uint32_t>> welds;
      for (const auto& p : scene.mesh.seamPairs) welds.emplace_back(p.a, p.b);
      renderer_->setGarment(scene.mesh.triangles, welds, scene.mesh.rest.size());
      if (framedBody_ != scene.bodyType) {  // frame on launch and on a Body change (spec 4.2)
        framedBody_ = scene.bodyType;
        Vec3d lo = Vec3d::Constant(1e9), hi = Vec3d::Constant(-1e9);
        for (const auto& v : scene.mannequin.mesh.vertices) {
          lo = lo.cwiseMin(v);
          hi = hi.cwiseMax(v);
        }
        renderer_->camera().frame(lo, hi, render::Renderer::kVerticalFovRad,
                                  static_cast<double>(width()) / std::max(height(), 1));
      }
    }
    renderer_->updateGarment(frame_.positions);
  }
  if (renderer_->renderFrame() && ++framesPresented_ == 1) emit firstFramePresented();
}

void ViewportWidget::mousePressEvent(QMouseEvent* e) { lastMouse_ = e->position().toPoint(); }

void ViewportWidget::mouseMoveEvent(QMouseEvent* e) {
  if (!renderer_) return;
  const QPoint p = e->position().toPoint();
  const QPoint d = p - lastMouse_;
  lastMouse_ = p;
  if (e->buttons() & Qt::LeftButton) {
    renderer_->camera().orbit(d.x(), d.y());
  } else if (e->buttons() & (Qt::RightButton | Qt::MiddleButton)) {
    renderer_->camera().pan(d.x(), d.y(), height(), render::Renderer::kVerticalFovRad);
  }
}

void ViewportWidget::wheelEvent(QWheelEvent* e) {
  if (!renderer_) return;
  if (!e->pixelDelta().isNull()) {  // trackpad two-finger drag: pan
    renderer_->camera().pan(-e->pixelDelta().x(), -e->pixelDelta().y(), height(), render::Renderer::kVerticalFovRad);
  } else {  // mouse wheel: zoom
    renderer_->camera().zoom(e->angleDelta().y() / 120.0);
  }
}

bool ViewportWidget::event(QEvent* e) {
  if (e->type() == QEvent::NativeGesture && renderer_) {
    auto* g = static_cast<QNativeGestureEvent*>(e);
    if (g->gestureType() == Qt::ZoomNativeGesture) {  // pinch
      renderer_->camera().zoom(g->value() * 10.0);
      return true;
    }
  }
  return QWidget::event(e);
}

}  // namespace drape::app
