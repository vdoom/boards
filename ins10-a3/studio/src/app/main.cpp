#include "app/language.h"
#include "app/mainwindow.h"

#include <QApplication>
#include <QCommandLineParser>
#include <QOpenGLWidget>
#include <QPainter>
#include <QTimer>

int main(int argc, char *argv[]) {
    QApplication app(argc, argv);
    QApplication::setOrganizationName(QStringLiteral("INS10"));
    QApplication::setApplicationName(QStringLiteral("INS10 Studio"));
    QApplication::setApplicationVersion(QStringLiteral(INS10_STUDIO_VERSION));

    // The starting language, before any window exists: --lang, else the one saved in
    // the settings, else the system's. The Language menu switches while running.
    QString langOverride;
    const QStringList args = QCoreApplication::arguments();
    if (const qsizetype i = args.indexOf(QStringLiteral("--lang")); i >= 0 && i + 1 < args.size()) langOverride = args[i + 1];
    ins10::language::apply(langOverride.isEmpty() ? ins10::language::resolve(ins10::language::saved())
                           : langOverride == QLatin1String("uk") ? QStringLiteral("uk")
                                                                 : QStringLiteral("en"));

    QCommandLineParser parser;
    parser.setApplicationDescription(QStringLiteral("Desktop companion for the INS10-A3 board (ins10_ahrs firmware)."));
    parser.addHelpOption();
    parser.addVersionOption();
    const QCommandLineOption demo(QStringLiteral("demo"), QStringLiteral("Connect to the simulated board at start."));
    const QCommandLineOption page(QStringLiteral("page"), QStringLiteral("Page to show first."), QStringLiteral("name"));
    const QCommandLineOption quitAfter(QStringLiteral("quit-after"),
                                       QStringLiteral("Close after this many seconds (smoke tests)."),
                                       QStringLiteral("seconds"));
    const QCommandLineOption screenshot(QStringLiteral("screenshot"),
                                        QStringLiteral("With --quit-after: save a picture of the window before closing."),
                                        QStringLiteral("file"));
    const QCommandLineOption demoMotion(QStringLiteral("demo-motion"),
                                        QStringLiteral("Motion of the simulated board: demo, tumble, positions, warmup."),
                                        QStringLiteral("kind"));
    const QCommandLineOption demoSpeed(QStringLiteral("demo-speed"), QStringLiteral("Time factor of the simulated board."),
                                       QStringLiteral("factor"));
    const QCommandLineOption wizard(QStringLiteral("start-wizard"),
                                    QStringLiteral("Once connected, start a calibration wizard: imu, mag, thermal."),
                                    QStringLiteral("name"));
    const QCommandLineOption lang(QStringLiteral("lang"), QStringLiteral("Interface language for this run: en, uk."),
                                  QStringLiteral("code"));
    const QCommandLineOption stopAfter(QStringLiteral("stop-after"),
                                       QStringLiteral("With --start-wizard: stop and compute after this many seconds."),
                                       QStringLiteral("seconds"));
    parser.addOptions({demo, page, quitAfter, screenshot, demoMotion, demoSpeed, wizard, lang, stopAfter});
    parser.process(app);

    ins10::MainWindow window;
    window.show();
    if (parser.isSet(page)) window.showPage(parser.value(page));
    if (parser.isSet(demo)) {
        // Applied once the simulated board exists, i.e. when connected.
        auto once = std::make_shared<QMetaObject::Connection>();
        *once = QObject::connect(window.board(), &ins10::BoardModel::connectionChanged, &window,
                                 [&window, &parser, demoMotion, demoSpeed, wizard, stopAfter, once](ins10::ConnectionState state) {
                                     if (state != ins10::ConnectionState::Connected) return;
                                     QObject::disconnect(*once);
                                     if (parser.isSet(demoMotion)) window.setDemoMotion(parser.value(demoMotion));
                                     if (parser.isSet(demoSpeed)) window.setDemoSpeed(parser.value(demoSpeed).toDouble());
                                     if (parser.isSet(wizard)) {
                                         const QString which = parser.value(wizard);
                                         const double stop = parser.value(stopAfter).toDouble();
                                         QTimer::singleShot(300, &window, [&window, which, stop] { window.startWizard(which, stop); });
                                     }
                                 });
        window.connectDemo();
    }
    if (parser.isSet(quitAfter)) {
        const QString shot = parser.value(screenshot);
        QTimer::singleShot(static_cast<int>(parser.value(quitAfter).toDouble() * 1000.0), &window, [&window, shot] {
            if (!shot.isEmpty()) {
                // QWidget::grab() leaves out what QPainter drew over OpenGL; take
                // those widgets' own frames.
                QPixmap picture = window.grab();
                QPainter painter(&picture);
                for (QOpenGLWidget *gl : window.findChildren<QOpenGLWidget *>()) {
                    if (gl->isVisible()) painter.drawImage(gl->mapTo(&window, QPoint(0, 0)), gl->grabFramebuffer());
                }
                painter.end();
                picture.save(shot);
            }
            window.close();
        });
    }
    return app.exec();
}
