// DeviceSession against the simulated board: queue, timeouts, ERR, streams inside a
// reply, port loss, the $ATT rate around 'cal save' and on exit, reboot.
#include "device/controller.h"
#include "device/fakeboard.h"
#include "device/ports.h"
#include "device/session.h"

#include <QSettings>
#include <QSignalSpy>
#include <QTest>

using namespace ins10;

namespace {

FakeBoardConfig manualBoard() {
    FakeBoardConfig c;
    c.realTime = false;
    c.bootAttRateHz = 10;
    c.rebootDownMs = 150;
    return c;
}

SessionConfig fastSession() {
    SessionConfig c;
    c.settleMs = 10;
    c.identifyTimeoutMs = 300;
    c.commandTimeoutMs = 300;
    c.longCommandTimeoutMs = 500;
    c.closeTimeoutMs = 1000;
    c.batchIntervalMs = 5;
    c.reconnectPollMs = 20;
    c.reconnectTimeoutMs = 3000;
    return c;
}

// A session wired to one simulated board, with every command result collected.
struct Rig {
    FakeBoard board;
    FakeTransportFactory factory{&board};
    DeviceSession session;
    QList<CommandResult> results;
    StreamBatch streams;
    quint64 nextId = 1;

    explicit Rig(const FakeBoardConfig &config = manualBoard(), const SessionConfig &session = fastSession())
        : board(config), session(&factory, session) {
        QObject::connect(&this->session, &DeviceSession::commandFinished,
                         [this](const CommandResult &r) { results.append(r); });
        QObject::connect(&this->session, &DeviceSession::streamBatch,
                         [this](const StreamBatch &b) { streams.append(b); });
    }

    bool connect() {
        session.connectTo(kDemoPortName);
        return QTest::qWaitFor([this] { return session.state() == ConnectionState::Connected && session.queuedCommands() == 0; }, 3000);
    }

    quint64 send(const QString &text, CommandOrigin origin = CommandOrigin::App) {
        const quint64 id = nextId++;
        session.enqueue(id, text, origin, 0);
        return id;
    }

    std::optional<CommandResult> waitFor(quint64 id, int ms = 3000) {
        auto find = [&]() -> std::optional<CommandResult> {
            for (const CommandResult &r : results) {
                if (r.id == id) return r;
            }
            return std::nullopt;
        };
        (void)QTest::qWaitFor([&] { return find().has_value(); }, ms);
        return find();
    }

    bool idle(int ms = 3000) {
        return QTest::qWaitFor([this] { return session.queuedCommands() == 0; }, ms);
    }
};

} // namespace

class TestSession : public QObject {
    Q_OBJECT
private slots:
    void connectsAndTakesOverTheRate() {
        Rig rig;
        QSignalSpy identified(&rig.session, &DeviceSession::identified);
        QVERIFY(rig.connect());
        QCOMPARE(identified.size(), 1);
        QVERIFY(rig.session.info().isAhrs());
        QCOMPARE(rig.session.ownerAttRate(), 10);
        QCOMPARE(rig.board.attRate(), 50);
        // info, our rate, and mag/baro switched off because 'info' does not show them.
        QCOMPARE(rig.board.commands(), QStringList({"info", "stream att 50", "stream mag 0", "stream baro 0"}));
    }

    void queueKeepsOrderOneInFlight() {
        Rig rig;
        QVERIFY(rig.connect());
        rig.board.setReplyDelayMs(30);
        const quint64 a = rig.send(QStringLiteral("cal set declination 8.5"));
        const quint64 b = rig.send(QStringLiteral("cal show"));
        const quint64 c = rig.send(QStringLiteral("thermal show"));
        QCOMPARE(rig.session.queuedCommands(), 3);
        QCoreApplication::processEvents();
        // Only the first has reached the board.
        QCOMPARE(rig.board.commands().size(), 5);
        const auto rc = rig.waitFor(c);
        QVERIFY(rc && rc->ok);
        QCOMPARE(rig.results.size(), 3);
        QCOMPARE(rig.results[0].id, a);
        QCOMPARE(rig.results[1].id, b);
        QVERIFY(rig.results[1].lines.contains(QStringLiteral("cal set declination 8.5")));
        QCOMPARE(rig.results[1].lines.size(), 18);
        QVERIFY(rc->lines.front().startsWith(QStringLiteral("# observations=")));
    }

    void errorReply() {
        Rig rig;
        QVERIFY(rig.connect());
        const auto r = rig.waitFor(rig.send(QStringLiteral("cal set bogus 1")));
        QVERIFY(r && !r->ok && !r->timedOut);
        QCOMPARE(r->error, QStringLiteral("unknown field or wrong number of values"));
        const auto unknown = rig.waitFor(rig.send(QStringLiteral("frobnicate")));
        QCOMPARE(unknown->error, QStringLiteral("unknown command; try 'help'"));
    }

    void timeoutThenRecovers() {
        Rig rig;
        QVERIFY(rig.connect());
        rig.board.setMute(true);
        QElapsedTimer clock;
        clock.start();
        const auto r = rig.waitFor(rig.send(QStringLiteral("status")));
        QVERIFY(r && !r->ok && r->timedOut);
        QVERIFY(clock.elapsed() >= 280);
        rig.board.setMute(false);
        const auto next = rig.waitFor(rig.send(QStringLiteral("status")));
        QVERIFY(next && next->ok);
        QVERIFY(parseStatus(next->lines).imu.valid);
    }

    void tooLongCommandIsRefusedLocally() {
        Rig rig;
        QVERIFY(rig.connect());
        rig.board.clearCommands();
        const auto r = rig.waitFor(rig.send(QStringLiteral("cal set gyro_matrix") + QString(200, u'1')));
        QVERIFY(r && !r->ok);
        const auto words = rig.waitFor(rig.send(QStringLiteral("cal set gyro_matrix 1 2 3 4 5 6 7 8 9 10 11 12")));
        QVERIFY(words && !words->ok);
        QVERIFY(rig.board.commands().isEmpty());
    }

    void streamsInsideAReply() {
        Rig rig;
        QVERIFY(rig.connect());
        const QString att = QStringLiteral(
            "$ATT,1000,1.000000,0.000000,0.000000,0.000000,1.50,-2.50,3.50,86.50,0.2100,-0.1500,0.0900,0.03,0.21,31.40,00D");
        rig.board.setCommandOverride([att](const QString &cmd) -> std::optional<QStringList> {
            if (cmd != QLatin1String("thermal show")) return std::nullopt;
            return QStringList{QStringLiteral("# observations=5 range=20.0..21.0 C"), att,
                               QStringLiteral("$RAW,10,2,4,-8,12,8,0,-4,600"),
                               QStringLiteral("# INS10 AHRS 2.0.0 (pico2), library 2.0.0. Type 'help'."),
                               QStringLiteral("# x: c0=0.10000 c1=0.00000 c2=0.00000 dps"), att, QStringLiteral("OK")};
        });
        rig.streams.clear();
        const auto r = rig.waitFor(rig.send(QStringLiteral("thermal show")));
        QVERIFY(r && r->ok);
        QCOMPARE(r->lines, QStringList({"# observations=5 range=20.0..21.0 C", "# x: c0=0.10000 c1=0.00000 c2=0.00000 dps"}));
        QTRY_COMPARE(rig.streams.size(), 3);
        QCOMPARE(rig.streams[0].kind, LineKind::Att);
        QCOMPARE(rig.streams[1].kind, LineKind::Raw);
    }

    void streamsAndDamage() {
        Rig rig;
        QVERIFY(rig.connect());
        rig.session.setStreamRequest(QStringLiteral("test"), {1, true, true, 0});
        QVERIFY(rig.idle());
        QCOMPARE(rig.board.rawDiv(), 1);
        QVERIFY(rig.board.magStream() && rig.board.baroStream());
        rig.board.setLineDamage(0.05, 3);
        rig.streams.clear();
        rig.board.advance(2.0);
        QTRY_VERIFY(rig.session.counters().rawRecords > 900);
        const SessionCounters c = rig.session.counters();
        QVERIFY(c.damagedLines > 20);
        QVERIFY(c.lostSamples > 10);
        // Everything delivered as a record is a complete, valid record.
        int raw = 0;
        for (const ParsedLine &line : rig.streams) {
            QVERIFY(line.isStream() && !line.damaged);
            raw += line.kind == LineKind::Raw;
        }
        QCOMPARE(static_cast<quint64>(raw), c.rawRecords);
        // Commands still work through the noise.
        rig.board.setLineDamage(0.0);
        QVERIFY(rig.waitFor(rig.send(QStringLiteral("info")))->ok);
    }

    void streamArbitration() {
        Rig rig;
        QVERIFY(rig.connect());
        rig.session.setStreamRequest(QStringLiteral("plots"), {4, false, true, 0});
        rig.session.setStreamRequest(QStringLiteral("wizard"), {8, true, false, 10});
        QVERIFY(rig.idle());
        QCOMPARE(rig.board.rawDiv(), 8);          // higher priority wins
        QVERIFY(rig.board.magStream() && rig.board.baroStream());
        rig.session.setStreamRequest(QStringLiteral("wizard"), {});
        QVERIFY(rig.idle());
        QCOMPARE(rig.board.rawDiv(), 4);
        QVERIFY(!rig.board.magStream());
        rig.session.setStreamRequest(QStringLiteral("other"), {2, false, false, 0});
        QVERIFY(rig.idle());
        QCOMPARE(rig.board.rawDiv(), 2);          // same priority: more data
    }

    void calSaveStoresTheOwnersRate() {
        Rig rig;
        QVERIFY(rig.connect());
        QCOMPARE(rig.board.attRate(), 50);
        rig.board.clearCommands();
        const auto r = rig.waitFor(rig.send(QStringLiteral("cal save")));
        QVERIFY(r && r->ok);
        QVERIFY(rig.idle());
        QCOMPARE(rig.board.saveCount(), 1);
        QCOMPARE(rig.board.storedAttRate(), 10);  // not ours
        QCOMPARE(rig.board.attRate(), 50);        // and ours again afterwards
        QCOMPARE(rig.board.commands(), QStringList({"stream att 10", "cal save", "stream att 50"}));
        // Also when the save is typed in the console.
        rig.board.clearCommands();
        QVERIFY(rig.waitFor(rig.send(QStringLiteral("cal  save"), CommandOrigin::User))->ok);
        QVERIFY(rig.idle());
        QCOMPARE(rig.board.storedAttRate(), 10);
    }

    void userRateBecomesTheOwnersRate() {
        Rig rig;
        QVERIFY(rig.connect());
        QVERIFY(rig.waitFor(rig.send(QStringLiteral("stream att 20"), CommandOrigin::User))->ok);
        QCOMPARE(rig.session.ownerAttRate(), 20);
        QVERIFY(rig.waitFor(rig.send(QStringLiteral("cal save")))->ok);
        QVERIFY(rig.idle());
        QCOMPARE(rig.board.storedAttRate(), 20);
        rig.session.disconnectFromBoard();
        QTRY_COMPARE(rig.session.state(), ConnectionState::Disconnected);
        QCOMPARE(rig.board.attRate(), 20);
    }

    void disconnectRestoresTheBoard() {
        Rig rig;
        QVERIFY(rig.connect());
        rig.session.setStreamRequest(QStringLiteral("plots"), {4, true, true, 0});
        QVERIFY(rig.idle());
        QCOMPARE(rig.board.rawDiv(), 4);
        QSignalSpy states(&rig.session, &DeviceSession::stateChanged);
        rig.session.disconnectFromBoard();
        QTRY_COMPARE(rig.session.state(), ConnectionState::Disconnected);
        QCOMPARE(rig.board.rawDiv(), 0);
        QVERIFY(!rig.board.magStream());
        QVERIFY(!rig.board.baroStream());
        QCOMPARE(rig.board.attRate(), 10);
        QVERIFY(!rig.board.isOpen());
        QCOMPARE(states.first().at(0).value<ConnectionState>(), ConnectionState::Closing);
        QVERIFY(states.last().at(1).toString().isEmpty()); // restored cleanly
        // Commands after closing are refused, not lost in a queue.
        const auto r = rig.waitFor(rig.send(QStringLiteral("info")));
        QVERIFY(r && !r->ok);
    }

    void portLossFailsPendingCommands() {
        Rig rig;
        QVERIFY(rig.connect());
        rig.board.setReplyDelayMs(200);
        const quint64 a = rig.send(QStringLiteral("status"));
        const quint64 b = rig.send(QStringLiteral("info"));
        QTest::qWait(20);
        rig.board.unplug();
        const auto ra = rig.waitFor(a), rb = rig.waitFor(b);
        QVERIFY(ra && !ra->ok && rb && !rb->ok);
        QVERIFY(ra->error.contains(QStringLiteral("unplugged")));
        QCOMPARE(rig.session.state(), ConnectionState::Lost);
        QVERIFY(!rig.waitFor(rig.send(QStringLiteral("info")))->ok);
        // Plugging back in and connecting again works.
        rig.board.plugIn();
        rig.board.setReplyDelayMs(0);
        QVERIFY(rig.connect());
    }

    void rebootReconnects() {
        Rig rig;
        QVERIFY(rig.connect());
        QSignalSpy states(&rig.session, &DeviceSession::stateChanged);
        QVERIFY(rig.waitFor(rig.send(QStringLiteral("reboot")))->ok);
        QTRY_COMPARE_WITH_TIMEOUT(rig.session.state(), ConnectionState::Rebooting, 1000);
        QTRY_COMPARE_WITH_TIMEOUT(rig.session.state(), ConnectionState::Connected, 3000);
        QVERIFY(rig.idle());
        QCOMPARE(rig.board.attRate(), 50);
        bool sawRebooting = false;
        for (const auto &args : states) sawRebooting |= args.at(0).value<ConnectionState>() == ConnectionState::Rebooting;
        QVERIFY(sawRebooting);
    }

    void bootselDisconnects() {
        Rig rig;
        QVERIFY(rig.connect());
        QVERIFY(rig.waitFor(rig.send(QStringLiteral("bootsel")))->ok);
        QTRY_COMPARE_WITH_TIMEOUT(rig.session.state(), ConnectionState::Disconnected, 1000);
    }

    void notTheAhrsFirmware() {
        FakeBoardConfig config = manualBoard();
        config.firmware = QStringLiteral("ins10-test");
        Rig rig(config);
        rig.session.connectTo(kDemoPortName);
        QTRY_COMPARE_WITH_TIMEOUT(rig.session.state(), ConnectionState::NotAhrs, 2000);
        // The console may still talk to it; the application may not.
        rig.send(QStringLiteral("info"), CommandOrigin::App);
        QVERIFY(!rig.results.last().ok);
        rig.session.disconnectFromBoard();
        QCOMPARE(rig.session.state(), ConnectionState::Disconnected);
    }

    // Seen on the real board: a terminal had sent "info" and "status" without line
    // endings, so "infostatus" waited in the firmware's input line. The first command of
    // the next program is appended to it and answered with ERR unknown command.
    void leftoversInTheFirmwareInputLine() {
        Rig rig;
        rig.board.typeWithoutLineEnding("infostatus");
        QVERIFY(rig.connect());
        QVERIFY(rig.session.info().isAhrs());
        // The leftovers were ended by our empty line and answered before 'info' went out.
        QCOMPARE(rig.board.commands().mid(0, 2), QStringList({"infostatus", "info"}));
        QCOMPARE(rig.session.counters().damagedLines, 0u);
    }

    // The same when the leftovers arrive too late for the flush: the first 'info' gets a
    // wrong answer, and the session asks again instead of calling it another firmware.
    void oneBadAnswerToInfoIsNotAnotherFirmware() {
        Rig rig;
        int asked = 0;
        rig.board.setCommandOverride([&asked](const QString &command) -> std::optional<QStringList> {
            if (command != QLatin1String("info") || ++asked > 2) return std::nullopt;
            return asked == 1 ? QStringList{QStringLiteral("ERR unknown command; try 'help'")}
                              : QStringList{QStringLiteral("# fw=ins10-a"), QStringLiteral("OK")}; // cut short
        });
        QVERIFY(rig.connect());
        QVERIFY(rig.session.info().isAhrs());
        QCOMPARE(asked, 3);
    }

    void persistentlyWrongAnswersAreAnotherFirmware() {
        Rig rig;
        int asked = 0;
        rig.board.setCommandOverride([&asked](const QString &command) -> std::optional<QStringList> {
            if (command != QLatin1String("info")) return std::nullopt;
            ++asked;
            return QStringList{QStringLiteral("ERR unknown command; try 'help'")};
        });
        rig.session.connectTo(kDemoPortName);
        QTRY_COMPARE_WITH_TIMEOUT(rig.session.state(), ConnectionState::NotAhrs, 2000);
        QCOMPARE(asked, 3);
    }

    void lateBannerIsNotAReply() {
        FakeBoardConfig config = manualBoard();
        config.bannerDelayMs = 30; // after the session has flushed the input
        SessionConfig sc = fastSession();
        sc.settleMs = 5;
        Rig rig(config, sc);
        rig.board.setReplyDelayMs(60);
        QVERIFY(rig.connect());
        QCOMPARE(rig.session.info().lines.size(), 7);
    }

    void rememberedOwnerRateAfterABrokenConnection() {
        QCoreApplication::setOrganizationName(QStringLiteral("INS10-tests"));
        QCoreApplication::setApplicationName(QStringLiteral("tst_session"));
        SessionConfig sc = fastSession();
        sc.persistGroup = QStringLiteral("pendingRestore");
        Rig rig(manualBoard(), sc);
        QVERIFY(rig.connect());
        QCOMPARE(rig.board.attRate(), 50);
        rig.board.unplug();         // the board kept running at our rate
        QTRY_COMPARE(rig.session.state(), ConnectionState::Lost);
        rig.board.plugIn();
        QVERIFY(rig.connect());
        QCOMPARE(rig.session.ownerAttRate(), 10); // not the 50 'info' showed
        rig.session.disconnectFromBoard();
        QTRY_COMPARE(rig.session.state(), ConnectionState::Disconnected);
        QCOMPARE(rig.board.attRate(), 10);
        QSettings().remove(QStringLiteral("pendingRestore"));
    }

    void controllerInAWorkerThread() {
        FakeBoardConfig config;
        config.realTime = true;
        DeviceController controller(std::make_unique<DefaultTransportFactory>(config), DeviceController::Threading::Worker,
                                    fastSession());
        QSignalSpy batches(&controller, &DeviceController::streamBatch);
        controller.connectTo(kDemoPortName);
        QTRY_VERIFY_WITH_TIMEOUT(controller.isConnected(), 3000);
        bool replied = false;
        QStringList lines;
        controller.command(QStringLiteral("status"), [&](const CommandResult &r) {
            replied = r.ok;
            lines = r.lines;
        });
        QTRY_VERIFY_WITH_TIMEOUT(replied, 3000);
        QVERIFY(parseStatus(lines).flags.valid);
        QTRY_VERIFY_WITH_TIMEOUT(batches.size() > 3, 3000); // $ATT at 50 Hz in real time
        bool done = false, ok = false;
        controller.commandSequence({QStringLiteral("cal set declination 3"), QStringLiteral("cal set bogus 1"),
                                    QStringLiteral("cal show")},
                                   [&](bool success, const QString &error, const QList<CommandResult> &results) {
                                       done = true;
                                       ok = success;
                                       QVERIFY(error.contains(QStringLiteral("bogus")));
                                       QCOMPARE(results.size(), 2);
                                   });
        QTRY_VERIFY(done);
        QVERIFY(!ok);
        QVERIFY(controller.shutdown(3000));
        QCOMPARE(controller.state(), ConnectionState::Disconnected);
    }
};

QTEST_GUILESS_MAIN(TestSession)
#include "tst_session.moc"
