#include "app/mainwindow.h"

#include "app/calibrationpage.h"
#include "app/calwizards.h"
#include "app/connectionpanel.h"
#include "app/consolepage.h"
#include "app/firstrunpage.h"
#include "app/language.h"
#include "app/overviewpage.h"
#include "app/plotspage.h"
#include "app/recordpage.h"
#include "app/settingsdialog.h"
#include "app/statuspage.h"
#include "app/uiutil.h"
#include "device/fakeboard.h"

#include <QActionGroup>
#include <QApplication>
#include <QCloseEvent>
#include <QLabel>
#include <QMenuBar>
#include <QMessageBox>
#include <QSettings>
#include <QStatusBar>
#include <QTabWidget>
#include <QTimer>
#include <QToolBar>

namespace ins10 {

namespace {
// A menu entry whose text follows the interface language.
QAction *translatedAction(QMenu *menu, std::function<QString()> text, std::function<void()> triggered) {
    QAction *action = menu->addAction(QString());
    QObject::connect(action, &QAction::triggered, menu, std::move(triggered));
    ui::translated(action, [action, text] { action->setText(text()); });
    return action;
}

QMenu *translatedMenu(QMenu *parent, std::function<QString()> title) {
    QMenu *menu = parent->addMenu(QString());
    ui::translated(menu, [menu, title] { menu->setTitle(title()); });
    return menu;
}

QMenu *translatedMenu(QMenuBar *bar, std::function<QString()> title) {
    QMenu *menu = bar->addMenu(QString());
    ui::translated(menu, [menu, title] { menu->setTitle(title()); });
    return menu;
}
} // namespace

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent) {
    SessionConfig config;
    config.viewAttRateHz = SettingsDialog::viewAttRate();
    config.persistGroup = QStringLiteral("pendingRestore");
    m_device = new DeviceController(std::make_unique<DefaultTransportFactory>(), DeviceController::Threading::Worker,
                                    config, this);
    m_board = new BoardModel(m_device, this);

    m_panel = new ConnectionPanel(m_board, this);
    auto *bar = new QToolBar(this);
    bar->setObjectName(QStringLiteral("connectionBar"));
    bar->setMovable(false);
    bar->addWidget(m_panel);
    addToolBar(Qt::TopToolBarArea, bar);
    ui::translated(this, [this, bar] {
        setWindowTitle(tr("INS10 Studio"));
        bar->setWindowTitle(tr("Connection"));
    });

    m_tabs = new QTabWidget(this);
    m_tabs->setObjectName(QStringLiteral("pages"));
    m_tabs->setDocumentMode(true);
    setCentralWidget(m_tabs);
    addPage(new OverviewPage(m_board, this), QStringLiteral("overview"), [] { return tr("Overview"); });
    addPage(new PlotsPage(m_board, this), QStringLiteral("plots"), [] { return tr("Plots"); });
    addPage(new FirstRunPage(m_board, this), QStringLiteral("firstrun"), [] { return tr("First run"); });
    addPage(new StatusPage(m_board, this), QStringLiteral("status"), [] { return tr("Status"); });
    m_calibration = new CalibrationPage(m_board, this);
    addPage(m_calibration, QStringLiteral("calibration"), [] { return tr("Calibration"); });
    addPage(new RecordPage(m_board, this), QStringLiteral("record"), [] { return tr("Recording"); });
    addPage(new ConsolePage(m_board, this), QStringLiteral("console"), [] { return tr("Console"); });

    buildMenus();
    connect(m_board, &BoardModel::connectionChanged, this, [this](ConnectionState state, const QString &detail) {
        m_connection = {state, detail};
        showConnectionMessage();
        if (state == ConnectionState::NotAhrs) {
            QMessageBox::warning(this, tr("Not the AHRS firmware"), detail);
        }
    });
    // The connection message, while it is still shown, in the new language.
    connect(language::notifier(), &language::Notifier::changed, this, [this] {
        if (statusBar()->currentMessage() == m_connectionMessage) showConnectionMessage();
    });

    QSettings settings;
    restoreGeometry(settings.value(QStringLiteral("window/geometry")).toByteArray());
    if (!settings.contains(QStringLiteral("window/geometry"))) resize(1360, 860);
    const int tab = settings.value(QStringLiteral("window/tab"), 0).toInt();
    if (tab >= 0 && tab < m_tabs->count()) m_tabs->setCurrentIndex(tab);
}

MainWindow::~MainWindow() = default;

void MainWindow::addPage(QWidget *page, const QString &name, std::function<QString()> title) {
    page->setObjectName(name);
    const int index = m_tabs->addTab(page, QString());
    ui::translated(m_tabs, [this, index, title] { m_tabs->setTabText(index, title()); });
}

void MainWindow::showConnectionMessage() {
    if (!m_connection) return;
    const auto &[state, detail] = *m_connection;
    m_connectionMessage = connectionStateText(state);
    if (!detail.isEmpty()) m_connectionMessage += QStringLiteral(": ") + detail;
    statusBar()->showMessage(m_connectionMessage);
}

void MainWindow::showPage(const QString &name) {
    for (int i = 0; i < m_tabs->count(); ++i) {
        if (m_tabs->widget(i)->objectName() == name) m_tabs->setCurrentIndex(i);
    }
}

void MainWindow::connectDemo() { m_panel->connectDemo(); }

void MainWindow::withDemoBoard(std::function<void(FakeBoard *)> action) {
    auto *factory = dynamic_cast<DefaultTransportFactory *>(m_device->factory());
    if (!factory) return;
    // The simulated board lives in the device thread and exists once connected.
    QMetaObject::invokeMethod(m_device->session(), [factory, action] {
        if (FakeBoard *board = factory->demoBoard()) action(board);
    });
}

void MainWindow::setDemoMotion(const QString &kind) {
    using K = FakeBoardConfig::MotionKind;
    const K motion = kind == QLatin1String("tumble")      ? K::Tumble
                     : kind == QLatin1String("positions") ? K::MultiPosition
                     : kind == QLatin1String("warmup")    ? K::Still
                                                          : K::Demo;
    withDemoBoard([motion](FakeBoard *board) { board->setMotion(motion); });
}

void MainWindow::setDemoSpeed(double factor) {
    withDemoBoard([factor](FakeBoard *board) { board->setSpeed(factor); });
}

void MainWindow::startWizard(const QString &which, double stopAfterSeconds) {
    showPage(QStringLiteral("calibration"));
    WizardWidget *wizard = nullptr;
    if (which == QLatin1String("imu")) {
        m_calibration->showTab(1);
        wizard = m_calibration->imuWizard();
    } else if (which == QLatin1String("mag")) {
        m_calibration->showTab(2);
        wizard = m_calibration->magWizard();
    } else if (which == QLatin1String("thermal")) {
        m_calibration->showTab(3);
        wizard = m_calibration->thermalWizard();
    }
    if (!wizard) return;
    wizard->run()->start();
    if (stopAfterSeconds > 0.0) {
        QTimer::singleShot(static_cast<int>(stopAfterSeconds * 1000.0), wizard, [wizard] {
            wizard->run()->stop();
            if (wizard->run()->state() == CalibrationRun::State::Recorded) wizard->run()->compute();
        });
    }
}

void MainWindow::buildMenus() {
    QMenu *file = translatedMenu(menuBar(), [] { return tr("&File"); });
    translatedAction(file, [] { return tr("&Settings..."); }, [this] {
        SettingsDialog dialog(this);
        connect(&dialog, &SettingsDialog::viewAttRateChanged, m_device, &DeviceController::setViewAttRate);
        if (dialog.exec() == QDialog::Accepted) language::apply(language::resolve(language::saved()));
    });
    file->addSeparator();
    translatedAction(file, [] { return tr("&Quit"); }, [this] { close(); })->setShortcut(QKeySequence::Quit);

    QMenu *device = translatedMenu(menuBar(), [] { return tr("&Board"); });
    translatedAction(device, [] { return tr("Connect the &simulated board"); }, [this] { connectDemo(); });
    translatedAction(device, [] { return tr("&Disconnect"); }, [this] { m_device->disconnectFromBoard(); });
    device->addSeparator();
    translatedAction(device, [] { return tr("Restart the &filter..."); }, [this] {
        confirmAndSend(QStringLiteral("reset"), tr("Restart the filter"),
                       tr("The filter starts again from initial alignment. Leave the board still for 2 s. Restart?"));
    });
    translatedAction(device, [] { return tr("&Reboot the Pico..."); }, [this] {
        confirmAndSend(QStringLiteral("reboot"), tr("Reboot"),
                       tr("The Pico restarts; the application reconnects when the port comes back. Unsaved "
                          "calibration changes are lost. Reboot?"));
    });
    translatedAction(device, [] { return tr("Enter the USB &bootloader..."); }, [this] {
        confirmAndSend(QStringLiteral("bootsel"), tr("Bootloader"),
                       tr("The Pico switches to its USB bootloader (BOOTSEL) for flashing new firmware and the port "
                          "disappears. Continue?"));
    });
    device->addSeparator();
    QMenu *demo = translatedMenu(device, [] { return tr("Simulated &board"); });
    auto *motions = new QActionGroup(this);
    const QList<std::pair<QString, std::function<QString()>>> kinds = {
        {QStringLiteral("demo"), [] { return tr("Demo choreography"); }},
        {QStringLiteral("tumble"), [] { return tr("Slow tumble (for the magnetometer wizard)"); }},
        {QStringLiteral("positions"), [] { return tr("Rest positions (for the accelerometer and gyro wizard)"); }},
        {QStringLiteral("warmup"), [] { return tr("Lying still, warming up (for the temperature wizard)"); }}};
    for (const auto &[kind, text] : kinds) {
        QAction *a = translatedAction(demo, text, [this, kind] { setDemoMotion(kind); });
        a->setCheckable(true);
        a->setChecked(kind == QLatin1String("demo"));
        motions->addAction(a);
    }
    demo->addSeparator();
    auto *speeds = new QActionGroup(this);
    for (double factor : {1.0, 10.0, 50.0}) {
        QAction *a = translatedAction(demo, [factor] { return tr("Time x%1").arg(factor); }, [this, factor] { setDemoSpeed(factor); });
        a->setCheckable(true);
        a->setChecked(factor == 1.0);
        speeds->addAction(a);
    }

    // Each language under its own name, so that it can be found from any other.
    QMenu *languageMenu = translatedMenu(menuBar(), [] { return tr("&Language"); });
    languageMenu->setObjectName(QStringLiteral("languageMenu"));
    auto *languages = new QActionGroup(this);
    const QList<std::pair<QString, std::function<QString()>>> choices = {
        {QString(), [] { return tr("As the system (%1)").arg(language::nativeName(language::resolve(QString()))); }},
        {QStringLiteral("en"), [] { return language::nativeName(QStringLiteral("en")); }},
        {QStringLiteral("uk"), [] { return language::nativeName(QStringLiteral("uk")); }}};
    for (const auto &[code, text] : choices) {
        QAction *a = translatedAction(languageMenu, text, [this, code] { chooseLanguage(code); });
        a->setObjectName(QStringLiteral("language_") + (code.isEmpty() ? QStringLiteral("system") : code));
        a->setData(code);
        a->setCheckable(true);
        languages->addAction(a);
    }
    auto markSaved = [languages] {
        const QString saved = language::saved();
        for (QAction *a : languages->actions()) a->setChecked(a->data().toString() == saved);
    };
    markSaved();
    connect(languageMenu, &QMenu::aboutToShow, this, markSaved);

    QMenu *help = translatedMenu(menuBar(), [] { return tr("&Help"); });
    translatedAction(help, [] { return tr("&About"); }, [this] {
        QMessageBox::about(this, tr("About INS10 Studio"),
                           tr("<b>INS10 Studio %1</b><p>Orientation display, diagnostics and calibration for the "
                              "INS10-A3 board running the ins10_ahrs firmware.</p><p>Qt %2. Linear algebra: Eigen.</p>")
                               .arg(QApplication::applicationVersion(), QString::fromLatin1(qVersion())));
    });
}

void MainWindow::chooseLanguage(const QString &choice) {
    language::save(choice);
    language::apply(language::resolve(choice));
}

void MainWindow::confirmAndSend(const QString &command, const QString &title, const QString &question) {
    if (!m_device->isConnected()) return;
    if (QMessageBox::question(this, title, question) != QMessageBox::Yes) return;
    m_device->command(command, [this, title](const CommandResult &r) {
        if (!r.ok) QMessageBox::warning(this, title, tr("The board refused: %1").arg(r.error));
    });
}

void MainWindow::closeEvent(QCloseEvent *event) {
    QSettings settings;
    settings.setValue(QStringLiteral("window/geometry"), saveGeometry());
    settings.setValue(QStringLiteral("window/tab"), m_tabs->currentIndex());
    if (!m_closing) {
        m_closing = true;
        // Give the board its own settings back before the port closes.
        statusBar()->showMessage(tr("Restoring the board's settings..."));
        m_device->shutdown(4000);
    }
    event->accept();
}

} // namespace ins10
