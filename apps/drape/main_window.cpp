#include "main_window.hpp"

#include <QAction>
#include <QApplication>
#include <QMessageBox>
#include <QStandardItemModel>
#include <QStatusBar>
#include <QToolBar>

#include <string>

#include "self_test.hpp"
#include "viewport_widget.hpp"

namespace drape::app {

namespace {

QString phaseText(sim::DrapeState s) {
  switch (s) {
    case sim::DrapeState::Assembling: return "Assembling";
    case sim::DrapeState::Settling: return "Settling";
    case sim::DrapeState::Settled: return "Settled";
    case sim::DrapeState::Failed: return "Failed";
    case sim::DrapeState::TimedOut: return "Stopped";
  }
  return "";
}

}  // namespace

MainWindow::MainWindow(const std::filesystem::path& resources, const std::optional<std::filesystem::path>& selftestPng,
                       const QElapsedTimer& sinceLaunch)
    : resources_(resources) {
  setWindowTitle("Drape");
  session::SimConfig config;
  config.body = avatar::BodyType::Male;
  config.quality = SimQuality::Standard;
  config.backend = sim::SolverBackend::Metal;
  config.mannequinDir = resources_ / "mannequins";
  controller_ = std::make_unique<session::SimController>(config);

  viewport_ = new ViewportWidget(*controller_, render::RendererAssets{resources_ / "materials", resources_ / "ibl"});
  setCentralWidget(viewport_);
  connect(viewport_, &ViewportWidget::rendererFailed, this,
          [this](const QString& why) { fatal("Filament renderer", why); });

  QToolBar* bar = addToolBar("Drape");
  bar->setMovable(false);
  QAction* simulate = bar->addAction("Simulate");
  simulate->setCheckable(true);
  simulate->setChecked(true);
  connect(simulate, &QAction::toggled, this, [this](bool on) { controller_->setRunning(on); });
  QAction* reset = bar->addAction("Reset drape");
  connect(reset, &QAction::triggered, this, [this] { controller_->reset(); });
  bar->addSeparator();

  auto combo = [&](const QString& label, const QStringList& items, int current) {
    bar->addWidget(new QLabel(" " + label + " "));
    auto* c = new QComboBox;
    c->addItems(items);
    c->setCurrentIndex(current);
    bar->addWidget(c);
    connect(c, &QComboBox::currentIndexChanged, this, [this] { applyConfig(); });
    return c;
  };
  quality_ = combo("Quality", {"Draft", "Standard", "Fine"}, 1);
  body_ = combo("Body", {"Male", "Female"}, 0);
  solver_ = combo("Solver", {"GPU", "CPU"}, 0);

  phase_ = new QLabel;
  fps_ = new QLabel;
  particles_ = new QLabel;
  message_ = new QLabel;
  statusBar()->addWidget(phase_);
  statusBar()->addWidget(fps_);
  statusBar()->addWidget(particles_);
  statusBar()->addWidget(message_, 1);
  connect(&statusTimer_, &QTimer::timeout, this, [this] { refreshStatus(); });
  statusTimer_.start(100);

  if (selftestPng) selftest_ = std::make_unique<SelfTest>(*this, *selftestPng, sinceLaunch);
}

MainWindow::~MainWindow() = default;

void MainWindow::applyConfig() {
  session::SimConfig config;
  config.quality = static_cast<SimQuality>(quality_->currentIndex());  // Draft, Standard, Fine
  config.body = body_->currentIndex() == 0 ? avatar::BodyType::Male : avatar::BodyType::Female;
  config.backend = solver_->currentIndex() == 0 ? sim::SolverBackend::Metal : sim::SolverBackend::Cpu;
  config.mannequinDir = resources_ / "mannequins";
  controller_->configure(config);
}

void MainWindow::refreshStatus() {
  const session::SimStatus s = controller_->status();
  phase_->setText(s.particles == 0 && s.state != sim::DrapeState::Failed ? "Loading" : phaseText(s.state));
  fps_->setText(QString::number(static_cast<int>(s.fps)) + " fps");
  particles_->setText(QString::number(s.particles) + " particles");
  message_->setText(QString::fromStdString(s.message));
  if (!s.gpuAvailable) {
    auto* model = qobject_cast<QStandardItemModel*>(solver_->model());
    if (model && model->item(0)->isEnabled()) {
      // Block first: disabling the current item can move the selection, and a stray currentIndexChanged would
      // rebuild the drape a second time.
      const QSignalBlocker block(solver_);
      model->item(0)->setEnabled(false);
      solver_->setCurrentIndex(1);
    }
  }
  if (s.state == sim::DrapeState::Failed && s.particles == 0 && !s.message.empty() && !selftest_) {
    // The scene could not be built (spec 6: bad mannequin or garment assets).
    fatal("Scene", QString::fromStdString(s.message));
  }
}

void MainWindow::fatal(const QString& component, const QString& message) {
  statusTimer_.stop();
  // The self-test reports failures itself and must never wait on a modal.
  if (!selftest_) QMessageBox::critical(this, "Drape", component + ": " + message);
  // Deferred: the renderer can fail inside window.show(), before app.exec() runs, where exit() would be a no-op.
  QTimer::singleShot(0, qApp, [] { QApplication::exit(1); });
}

}  // namespace drape::app
