// Drape: the Phase B Mac app shell. `Drape --selftest <out.png>` runs the scripted self-test (Phase B spec 7.2).
#include <QApplication>
#include <QCommandLineParser>
#include <QElapsedTimer>
#include <QMessageBox>

#include <exception>
#include <filesystem>
#include <optional>

#include "main_window.hpp"

int main(int argc, char** argv) {
  QElapsedTimer sinceLaunch;
  sinceLaunch.start();
  QApplication app(argc, argv);
  QApplication::setApplicationName("Drape");

  QCommandLineParser parser;
  parser.addHelpOption();
  QCommandLineOption selftest("selftest", "Drape the tee, save a screenshot to <png> and exit (0 = settled).", "png");
  parser.addOption(selftest);
  parser.process(app);

  const std::filesystem::path resources =
      std::filesystem::path(QCoreApplication::applicationDirPath().toStdString()) / ".." / "Resources";
  std::optional<std::filesystem::path> selftestPng;
  if (parser.isSet(selftest)) selftestPng = parser.value(selftest).toStdString();
  try {
    drape::app::MainWindow window(resources, selftestPng, sinceLaunch);
    window.resize(1280, 800);
    window.show();
    return app.exec();
  } catch (const std::exception& e) {
    QMessageBox::critical(nullptr, "Drape", QString::fromStdString(e.what()));
    return 1;
  }
}
