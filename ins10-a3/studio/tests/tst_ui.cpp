// Widgets end to end: the main window against the simulated board, the recording
// page, and logs written by the application read back by the Python tool
// (`ins10.py magcal --from-log`), when Python with numpy and scipy is installed.
#include "app/calibrationpage.h"
#include "app/calwizards.h"
#include "app/glscene.h"
#include "app/language.h"
#include "app/mainwindow.h"
#include "app/plotspage.h"
#include "app/plotwidget.h"
#include "app/recordpage.h"
#include "app/xyplot.h"
#include "core/logfile.h"
#include "device/calibrationrun.h"
#include "device/fakeboard.h"
#include "device/ports.h"
#include "fixture_dir.h"

#include <QAbstractSpinBox>
#include <QAction>
#include <QApplication>
#include <QCheckBox>
#include <QComboBox>
#include <QDir>
#include <QDoubleSpinBox>
#include <QFile>
#include <QGroupBox>
#include <QLabel>
#include <QLineEdit>
#include <QMenu>
#include <QProcess>
#include <QPushButton>
#include <QRegularExpression>
#include <QScopeGuard>
#include <QSettings>
#include <QSpinBox>
#include <QStandardPaths>
#include <QStatusBar>
#include <QTabWidget>
#include <QTableWidget>
#include <QTemporaryDir>
#include <QTest>
#include <QToolBar>
#include <QTreeWidget>
#include <QTreeWidgetItemIterator>
#include <QXmlStreamReader>

using namespace ins10;

namespace {

QString pythonTool() {
    return QDir::cleanPath(QString::fromUtf8(INS10_FIXTURE_DIR) + QStringLiteral("/../../../firmware/tools/ins10.py"));
}

QString findPython() {
    for (const char *name : {"python", "python3"}) {
        const QString path = QStandardPaths::findExecutable(QLatin1String(name));
        if (path.isEmpty()) continue;
        QProcess probe;
        probe.start(path, {QStringLiteral("-c"), QStringLiteral("import numpy, scipy")});
        if (probe.waitForFinished(30000) && probe.exitCode() == 0) return path;
    }
    return {};
}

// Every piece of interface text in the window that the code sets: titles, labels,
// buttons, tool tips, tabs, menus, list and table headers, combo box items, plot
// titles and trace names, the 3D view's labels and the status bar. Not the serial
// port names (they come from the system) and not text typed into fields.
QStringList interfaceTexts(QMainWindow *window) {
    QStringList ports;
    for (const PortInfo &p : availablePorts()) ports.append(p.name);
    QStringList out{window->windowTitle(), window->statusBar()->currentMessage()};
    QList<QWidget *> widgets = window->findChildren<QWidget *>();
    for (QWidget *w : widgets) {
        out << w->toolTip();
        if (auto *l = qobject_cast<QLabel *>(w)) out << l->text();
        if (auto *b = qobject_cast<QAbstractButton *>(w)) out << b->text();
        if (auto *g = qobject_cast<QGroupBox *>(w)) out << g->title();
        if (auto *m = qobject_cast<QMenu *>(w)) out << m->title();
        if (auto *t = qobject_cast<QToolBar *>(w)) out << t->windowTitle();
        if (auto *e = qobject_cast<QLineEdit *>(w)) out << e->placeholderText();
        if (auto *s = qobject_cast<QDoubleSpinBox *>(w)) out << s->suffix();
        if (auto *s = qobject_cast<QSpinBox *>(w)) out << s->suffix();
        if (auto *t = qobject_cast<QTabWidget *>(w)) {
            for (int i = 0; i < t->count(); ++i) out << t->tabText(i);
        }
        if (auto *c = qobject_cast<QComboBox *>(w)) {
            for (int i = 0; i < c->count(); ++i) {
                if (!ports.contains(c->itemData(i).toString())) out << c->itemText(i);
            }
        }
        if (auto *t = qobject_cast<QTableWidget *>(w)) {
            for (int i = 0; i < t->columnCount(); ++i) {
                if (t->horizontalHeaderItem(i)) out << t->horizontalHeaderItem(i)->text();
            }
            if (t->columnCount() == 4 && t->rowCount() > 10) { // the calibration fields: meanings
                for (int r = 0; r < t->rowCount(); ++r) out << t->item(r, 3)->text();
            }
        }
        if (auto *t = qobject_cast<QTreeWidget *>(w)) {
            for (int c = 0; c < t->columnCount(); ++c) out << t->headerItem()->text(c);
            for (QTreeWidgetItemIterator it(t); *it; ++it) {
                for (int c = 0; c < t->columnCount(); ++c) out << (*it)->text(c);
            }
        }
        if (auto *p = qobject_cast<PlotWidget *>(w)) {
            out << p->title() << p->unit();
            for (const PlotTrace &trace : p->traces()) out << trace.label;
        }
        if (auto *x = qobject_cast<XyPlot *>(w)) out << x->titles();
        if (auto *g = qobject_cast<GlScene *>(w)) {
            for (const GlScene::Label &label : g->labels()) out << label.text;
        }
    }
    for (QAction *a : window->findChildren<QAction *>()) out << a->text();
    out.removeAll(QString());
    return out;
}

// Texts in Cyrillic, leaving out the name of the language itself (the Language menu
// shows every language under its own name).
QStringList cyrillic(const QStringList &texts) {
    static const QRegularExpression letters(QStringLiteral("[\\x{0400}-\\x{04FF}]"));
    QStringList out;
    for (const QString &t : texts) {
        if (QString(t).remove(QStringLiteral("Українська")).contains(letters)) out << t;
    }
    return out;
}

// Texts that are English sources with a different Ukrainian translation in the .ts:
// what a window that switched to Ukrainian still shows untranslated. Sources with
// arguments match with any argument; sources with fewer than four letters of their
// own (units, "%1 C") are left out.
QStringList untranslated(const QStringList &texts) {
    QFile ts(QString::fromUtf8(INS10_FIXTURE_DIR) + QStringLiteral("/../../i18n/ins10studio_uk.ts"));
    if (!ts.open(QIODevice::ReadOnly)) return {QStringLiteral("cannot read ") + ts.fileName()};
    QList<QRegularExpression> english;
    QXmlStreamReader xml(&ts);
    QString source;
    while (!xml.atEnd()) {
        if (!xml.readNextStartElement()) continue;
        if (xml.name() == QLatin1String("source")) source = xml.readElementText();
        else if (xml.name() == QLatin1String("translation")) {
            const QString translation = xml.readElementText();
            static const QRegularExpression arg(QStringLiteral("%\\d"));
            if (translation == source || QString(source).remove(arg).count(QRegularExpression(QStringLiteral("[A-Za-z]"))) < 4)
                continue;
            QString pattern = QRegularExpression::escape(source);
            pattern.replace(QRegularExpression(QStringLiteral("%\\d")), QStringLiteral(".*"));
            english.append(QRegularExpression(QRegularExpression::anchoredPattern(pattern),
                                              QRegularExpression::DotMatchesEverythingOption));
        }
    }
    QStringList out;
    for (const QString &t : texts) {
        for (const QRegularExpression &e : english) {
            if (e.match(t).hasMatch()) {
                out << t;
                break;
            }
        }
    }
    return out;
}

// Runs `ins10.py magcal --from-log <log>` and returns the fields it printed.
std::optional<CalibrationSet> pythonMagcal(const QString &python, const QString &log, QString *output) {
    QProcess p;
    QProcessEnvironment env = QProcessEnvironment::systemEnvironment();
    env.insert(QStringLiteral("PYTHONDONTWRITEBYTECODE"), QStringLiteral("1")); // nothing written next to the tool
    p.setProcessEnvironment(env);
    p.start(python, {pythonTool(), QStringLiteral("magcal"), QStringLiteral("--from-log"), log});
    if (!p.waitForFinished(180000)) return std::nullopt;
    *output = QString::fromUtf8(p.readAllStandardOutput() + p.readAllStandardError());
    if (p.exitCode() != 0) return std::nullopt;
    return parseCalShow(*output);
}

struct Bench {
    std::unique_ptr<FakeBoard> board;
    DeviceController device;
    BoardModel model;
    explicit Bench(FakeBoardConfig config)
        : board(std::make_unique<FakeBoard>(config)),
          device(std::make_unique<FakeTransportFactory>(board.get()), DeviceController::Threading::SameThread, [] {
              SessionConfig c;
              c.settleMs = 5;
              c.batchIntervalMs = 5;
              return c;
          }()),
          model(&device) {}
    bool connect() {
        device.connectTo(kDemoPortName);
        return QTest::qWaitFor([this] { return device.isConnected() && device.session()->queuedCommands() == 0; }, 5000);
    }
    void run(double seconds) {
        for (double t = 0; t < seconds; t += 0.5) {
            board->advance(0.5);
            QCoreApplication::processEvents();
        }
        QTest::qWait(30);
    }
};

FakeBoardConfig tumbleBoard() {
    FakeBoardConfig c;
    c.realTime = false;
    c.motion = FakeBoardConfig::MotionKind::Tumble;
    c.tumbleRateDps = 50.0;
    c.motionSeed = 21;
    return c;
}

} // namespace

class TestUi : public QObject {
    Q_OBJECT
private slots:
    void initTestCase() {
        QCoreApplication::setOrganizationName(QStringLiteral("INS10-tests"));
        QCoreApplication::setApplicationName(QStringLiteral("tst_ui"));
    }

    void mainWindowWithTheDemoBoard() {
        MainWindow window;
        window.resize(1300, 850);
        window.show();
        window.connectDemo();
        QTRY_VERIFY_WITH_TIMEOUT(window.board()->isConnected(), 5000);
        auto *tabs = window.findChild<QTabWidget *>(QStringLiteral("pages"));
        QVERIFY(tabs);
        // Overview, Plots, First run, Status, Calibration, Recording, Console.
        QCOMPARE(tabs->count(), 7);
        for (int i = 0; i < tabs->count(); ++i) {
            tabs->setCurrentIndex(i);
            QTest::qWait(250);
        }
        QTRY_VERIFY_WITH_TIMEOUT(window.board()->lastAtt().has_value(), 3000);
        window.close(); // restores the board's settings and disconnects
        QCOMPARE(window.board()->device()->state(), ConnectionState::Disconnected);
    }

    // The Language menu switches the running window: no restart, the board stays
    // connected, and no text is left in the other language, either way.
    void languageSwitchesWhileRunning() {
        language::save(QString());
        language::apply(QStringLiteral("en"));
        const auto restore = qScopeGuard([] {
            language::save(QString());
            language::apply(QStringLiteral("en"));
        });
        MainWindow window;
        window.resize(1300, 850);
        window.show();
        window.connectDemo();
        QTRY_VERIFY_WITH_TIMEOUT(window.board()->isConnected(), 5000);
        auto *tabs = window.findChild<QTabWidget *>(QStringLiteral("pages"));
        for (int i = 0; i < tabs->count(); ++i) {
            tabs->setCurrentIndex(i);
            QTest::qWait(150);
        }
        tabs->setCurrentIndex(3); // the status page, so that 'status' fills its tree
        QTRY_VERIFY_WITH_TIMEOUT(window.board()->status().flags.valid && window.board()->lastAtt().has_value(), 5000);
        QTest::qWait(1500);
        const QStringList english = interfaceTexts(&window);
        QVERIFY2(english.size() > 300, qPrintable(QString::number(english.size())));
        QVERIFY2(cyrillic(english).isEmpty(), qPrintable(cyrillic(english).join(u'\n')));
        // The check below recognises the English texts it is meant to catch.
        const qsizetype recognised = untranslated(english).size();
        QVERIFY2(recognised > 200, qPrintable(QString::number(recognised)));

        window.findChild<QAction *>(QStringLiteral("language_uk"))->trigger();
        QCOMPARE(language::saved(), QStringLiteral("uk"));
        QCOMPARE(language::active(), QStringLiteral("uk"));
        QCOMPARE(QCoreApplication::translate("ins10::MainWindow", "Overview"), QStringLiteral("Огляд")); // the .qm is there
        QTest::qWait(1500); // a few status polls and redraws in the new language
        QVERIFY(window.isVisible());
        QVERIFY(window.board()->isConnected());
        const QStringList ukrainian = interfaceTexts(&window);
        const QStringList left = untranslated(ukrainian);
        QVERIFY2(left.isEmpty(), qPrintable(left.join(u'\n')));
        QVERIFY2(cyrillic(ukrainian).size() > 200, qPrintable(QString::number(cyrillic(ukrainian).size())));

        window.findChild<QAction *>(QStringLiteral("language_en"))->trigger();
        QCOMPARE(language::active(), QStringLiteral("en"));
        QTest::qWait(1500);
        const QStringList back = interfaceTexts(&window);
        QVERIFY2(cyrillic(back).isEmpty(), qPrintable(cyrillic(back).join(u'\n')));
        QVERIFY(window.board()->isConnected());

        // "As the system" keeps no choice; Settings and the menu agree.
        window.findChild<QAction *>(QStringLiteral("language_system"))->trigger();
        QVERIFY(language::saved().isEmpty());
        QCOMPARE(language::active(), language::resolve(QString()));
        window.close();
    }

    // As on the real board (2026-11-18): the plots switched their streams off while
    // another page was open, which left holes in the magnetometer and barometer plots
    // that looked like the board dropping data. Only unticking a plot stops its stream.
    void plotStreamsStayOnWhileThePageIsHidden() {
        QSettings settings;
        settings.setValue(QStringLiteral("plots/show/mag"), true);
        settings.setValue(QStringLiteral("plots/show/baro"), true);
        FakeBoardConfig config;
        config.realTime = false;
        config.motion = FakeBoardConfig::MotionKind::Still;
        Bench bench(config);
        QVERIFY(bench.connect());
        PlotsPage page(&bench.model);
        page.show();
        const QStringList &sent = bench.board->commands();
        QTRY_VERIFY_WITH_TIMEOUT(sent.contains(QStringLiteral("stream mag 1")) && sent.contains(QStringLiteral("stream baro 1")),
                                 3000);
        page.hide();
        bench.run(0.5);
        const qsizetype mark = sent.size();
        const qsizetype mag = page.series().mag.size(), baro = page.series().baro.size();
        bench.run(3.0);
        QVERIFY2(!sent.mid(mark).contains(QStringLiteral("stream mag 0")), qPrintable(sent.mid(mark).join(u", ")));
        QVERIFY2(!sent.mid(mark).contains(QStringLiteral("stream baro 0")), qPrintable(sent.mid(mark).join(u", ")));
        QVERIFY2(page.series().mag.size() >= mag + 120, qPrintable(QString::number(page.series().mag.size() - mag)));  // 50 Hz
        QVERIFY2(page.series().baro.size() >= baro + 60, qPrintable(QString::number(page.series().baro.size() - baro))); // 25 Hz

        for (QCheckBox *box : page.findChildren<QCheckBox *>()) {
            if (box->text() == QLatin1String("Magnetic field magnitude (raw)")) box->setChecked(false);
        }
        QTRY_VERIFY_WITH_TIMEOUT(sent.mid(mark).contains(QStringLiteral("stream mag 0")), 3000);
        QVERIFY(!sent.mid(mark).contains(QStringLiteral("stream baro 0")));
        settings.remove(QStringLiteral("plots/show/mag"));
        settings.remove(QStringLiteral("plots/show/baro"));
    }

    void wizardButtons() {
        FakeBoardConfig config;
        config.realTime = false;
        config.motion = FakeBoardConfig::MotionKind::MultiPosition;
        config.positions = 10;
        Bench bench(config);
        QVERIFY(bench.connect());
        CalibrationPage page(&bench.model);
        page.show();
        ImuWizard *wizard = page.imuWizard();
        auto button = [wizard](const QString &text) {
            for (QPushButton *b : wizard->findChildren<QPushButton *>()) {
                if (b->text() == text) return b;
            }
            return static_cast<QPushButton *>(nullptr);
        };
        QPushButton *start = button(QStringLiteral("Start recording"));
        QPushButton *stop = button(QStringLiteral("Stop and compute"));
        QPushButton *tryIt = button(QStringLiteral("Try on the board"));
        QPushButton *revert = button(QStringLiteral("Cancel: restore the previous calibration"));
        QVERIFY(start && stop && tryIt && revert);
        QVERIFY(start->isEnabled() && !stop->isEnabled());
        QTest::mouseClick(start, Qt::LeftButton);
        QTRY_COMPARE(wizard->run()->state(), CalibrationRun::State::Recording);
        // The other wizards may not start meanwhile.
        QVERIFY(!page.magWizard()->findChildren<QPushButton *>().front()->isEnabled());
        bench.run(5.0 * config.positions + 4.0);
        QTest::mouseClick(stop, Qt::LeftButton);
        QTRY_VERIFY_WITH_TIMEOUT(wizard->run()->state() == CalibrationRun::State::Computed, 30000);
        QVERIFY(wizard->run()->result().ok);
        QVERIFY(tryIt->isEnabled());
        QTest::mouseClick(tryIt, Qt::LeftButton);
        QTRY_VERIFY_WITH_TIMEOUT(wizard->run()->tried(), 10000);
        QTRY_VERIFY(calFlags(bench.board->ramCalibration()) & CalAccel);
        QTRY_VERIFY(revert->isEnabled());
        QTest::mouseClick(revert, Qt::LeftButton);
        QTRY_VERIFY_WITH_TIMEOUT(!wizard->run()->tried(), 10000);
        QTRY_COMPARE(calFlags(bench.board->ramCalibration()), 0u);
    }

    void wizardLogReadByPython() {
        Bench bench(tumbleBoard());
        QVERIFY(bench.connect());
        CalibrationRun run(&bench.model, calib::Procedure::Mag);
        run.start();
        QTRY_COMPARE(run.state(), CalibrationRun::State::Recording);
        bench.run(120.0);
        run.stop();
        run.compute();
        QTRY_VERIFY_WITH_TIMEOUT(run.state() == CalibrationRun::State::Computed, 60000);
        QVERIFY(run.result().ok && run.result().mag);
        QTemporaryDir dir;
        const QString log = dir.filePath(QStringLiteral("magcal.log"));
        QVERIFY(run.saveRecording(log));

        const QString python = findPython();
        if (python.isEmpty()) QSKIP("Python with numpy and scipy not found");
        QString output;
        const auto fields = pythonMagcal(python, log, &output);
        QVERIFY2(fields.has_value(), qPrintable(output));
        const QList<double> bias = fields->value(QStringLiteral("mag_bias"));
        QCOMPARE(bias.size(), 3);
        for (int i = 0; i < 3; ++i) QVERIFY(std::abs(bias[i] - run.result().mag->bias[i]) < 0.02);
    }

    // The page used to keep saying "Connect a board to record." with a board connected.
    void recordPageHintGoesWhenABoardConnects() {
        FakeBoardConfig config;
        config.realTime = false;
        Bench bench(config);
        RecordPage page(&bench.model);
        auto *status = page.findChild<QLabel *>(QStringLiteral("recordStatus"));
        QVERIFY(status);
        QVERIFY(!status->text().isEmpty());
        QVERIFY(bench.connect());
        QTRY_VERIFY(status->text().isEmpty());
    }

    void recordPageLogReadByPython() {
        Bench bench(tumbleBoard());
        QVERIFY(bench.connect());
        RecordPage page(&bench.model);
        QTemporaryDir dir;
        const QString log = dir.filePath(QStringLiteral("record.log"));
        QString error;
        QVERIFY2(page.startRecording(log, &error), qPrintable(error));
        QTRY_VERIFY(bench.device.session()->queuedCommands() == 0);
        // Status polled meanwhile: its replies end up in the file, as in a terminal log.
        bench.model.refreshStatus();
        bench.run(120.0);
        page.stopRecording();
        QVERIFY(page.linesWritten() > 10000);
        QTRY_COMPARE(bench.board->rawDiv(), 0); // the recording's streams were handed back
        QTRY_VERIFY(!bench.board->magStream());

        const auto lines = readLogFile(log);
        QVERIFY(lines.has_value());
        QVERIFY(lines->front().startsWith(QStringLiteral("# fw=ins10-ahrs")));
        QVERIFY(lines->contains(QStringLiteral("cal set flags 0")));
        const auto rec = loadLog(log);
        QVERIFY(rec->imuRows() > 1000 && rec->magRows() > 1000);
        QCOMPARE(rec->lostSamples(), 0);

        const QString python = findPython();
        if (python.isEmpty()) QSKIP("Python with numpy and scipy not found");
        QString output;
        const auto fields = pythonMagcal(python, log, &output);
        QVERIFY2(fields.has_value(), qPrintable(output));
        const calib::WorkflowResult mine = calib::runMagCal(*rec, {}, calib::MagOptions{false, false});
        QVERIFY(mine.ok);
        const QList<double> bias = fields->value(QStringLiteral("mag_bias"));
        QCOMPARE(bias.size(), 3);
        for (int i = 0; i < 3; ++i) QVERIFY(std::abs(bias[i] - mine.mag->bias[i]) < 0.02);
    }
};

QTEST_MAIN(TestUi)
#include "tst_ui.moc"
