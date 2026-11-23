#pragma once

#include "device/boardmodel.h"
#include "device/controller.h"

#include <QMainWindow>

#include <functional>
#include <optional>
#include <utility>

class QTabWidget;

namespace ins10 {

class ConnectionPanel;

class MainWindow : public QMainWindow {
    Q_OBJECT
public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow() override;

    BoardModel *board() const { return m_board; }
    ConnectionPanel *connectionPanel() const { return m_panel; }
    void connectDemo();
    // Shows the page with this object name ("overview", "plots", ...).
    void showPage(const QString &name);
    // The simulated board: "demo", "tumble", "positions" or "warmup"; time factor.
    void setDemoMotion(const QString &kind);
    void setDemoSpeed(double factor);
    // Opens a calibration wizard ("imu", "mag", "thermal") and starts recording; with
    // stopAfterSeconds > 0 it stops and computes after that long.
    void startWizard(const QString &which, double stopAfterSeconds = 0.0);

    // Saves the interface language ("" for the system's, "en", "uk") and switches to
    // it at once; the board stays connected and every page keeps its state.
    void chooseLanguage(const QString &choice);

protected:
    void closeEvent(QCloseEvent *event) override;

private:
    void buildMenus();
    void addPage(QWidget *page, const QString &name, std::function<QString()> title);
    void showConnectionMessage();
    void confirmAndSend(const QString &command, const QString &title, const QString &question);

    void withDemoBoard(std::function<void(class FakeBoard *)> action);

    DeviceController *m_device;
    BoardModel *m_board;
    ConnectionPanel *m_panel;
    class CalibrationPage *m_calibration = nullptr;
    QTabWidget *m_tabs;
    bool m_closing = false;
    // The last connection change and the status bar text made of it.
    std::optional<std::pair<ConnectionState, QString>> m_connection;
    QString m_connectionMessage;
};

} // namespace ins10
