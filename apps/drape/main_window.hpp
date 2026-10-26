#pragma once

#include <QComboBox>
#include <QElapsedTimer>
#include <QLabel>
#include <QMainWindow>
#include <QTimer>

#include <filesystem>
#include <memory>
#include <optional>

#include "drape/session/sim_controller.hpp"

namespace drape::app {

class SelfTest;
class ViewportWidget;

// The Phase B window (spec 4.1): the viewport, a toolbar (Simulate, Reset drape, Quality, Body, Solver) and a status
// bar (phase, fps, particle count, message).
class MainWindow : public QMainWindow {
  Q_OBJECT

 public:
  MainWindow(const std::filesystem::path& resources, const std::optional<std::filesystem::path>& selftestPng,
             const QElapsedTimer& sinceLaunch);
  ~MainWindow() override;

  session::SimController& controller() { return *controller_; }
  ViewportWidget& viewport() { return *viewport_; }

 private:
  void applyConfig();
  void refreshStatus();
  void fatal(const QString& component, const QString& message);

  std::filesystem::path resources_;
  std::unique_ptr<session::SimController> controller_;
  ViewportWidget* viewport_ = nullptr;
  QComboBox* quality_ = nullptr;
  QComboBox* body_ = nullptr;
  QComboBox* solver_ = nullptr;
  QLabel* phase_ = nullptr;
  QLabel* fps_ = nullptr;
  QLabel* particles_ = nullptr;
  QLabel* message_ = nullptr;
  QTimer statusTimer_;
  std::unique_ptr<SelfTest> selftest_;
};

}  // namespace drape::app
