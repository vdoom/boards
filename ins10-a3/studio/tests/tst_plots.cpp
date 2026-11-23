// Plots: ten minutes at 200 Hz without the memory growing, drawing cost independent
// of the number of samples, CSV export; the 3D view where OpenGL 3.3 exists.
#include "app/boardview.h"
#include "app/plotwidget.h"
#include "core/streamseries.h"

#include <QApplication>
#include <QElapsedTimer>
#include <QImage>
#include <QSignalSpy>
#include <QTest>

#include <cmath>

using namespace ins10;

namespace {
ParsedLine attLine(double t, double roll) {
    ParsedLine line;
    line.kind = LineKind::Att;
    AttRecord r;
    r.tMs = static_cast<qint64>(std::llround(t * 1000.0));
    r.rollDeg = roll;
    r.pitchDeg = 0.5 * roll;
    r.yawDeg = -roll;
    r.headingDeg = 90.0;
    r.tempC = 30.0;
    line.data = r;
    return line;
}
} // namespace

class TestPlots : public QObject {
    Q_OBJECT
private slots:
    void tenMinutesAt200HzWithoutGrowth() {
        StreamSeries s;
        const size_t before = s.memoryBytes();
        // 15 minutes of $ATT at 200 Hz and $RAW at 128 Hz.
        const int attPerSecond = 200;
        for (int i = 0; i < 15 * 60 * attPerSecond; ++i) {
            const double t = i / 200.0;
            s.feed(attLine(t, 30.0 * std::sin(t)));
            if (i % 25 < 16) { // 128 of every 200
                ParsedLine raw;
                raw.kind = LineKind::Raw;
                RawRecord r;
                r.seq = static_cast<qint64>(t * 512.0);
                r.n = 4;
                r.accel = {0, 0, 4 * 32768};
                raw.data = r;
                s.feed(raw);
            }
        }
        QCOMPARE(s.memoryBytes(), before);
        QCOMPARE(s.att.size(), StreamSeries::kAttCapacity);
        QVERIFY(std::abs(s.att.lastTime() - (15 * 60 - 1 / 200.0)) < 1e-3);
        // At least ten minutes are kept.
        QVERIFY(s.att.lastTime() - s.att.firstTime() >= 600.0);
        QVERIFY(s.raw.lastTime() - s.raw.firstTime() >= 600.0);
        QCOMPARE(s.att.appended(), quint64(15 * 60 * 200));
    }

    void envelopeMatchesTheData() {
        TimeSeriesGroup g({QStringLiteral("v")}, 1000);
        for (int i = 0; i < 1000; ++i) g.append(i * 0.01, {std::sin(i * 0.01)});
        std::vector<TimeSeriesGroup::Bin> bins;
        g.envelope(0, 0.0, 10.0, 10, bins);
        int samples = 0;
        for (int c = 0; c < 10; ++c) {
            samples += bins[static_cast<size_t>(c)].count;
            const auto r = g.range(0, c, c + 0.999);
            QVERIFY(r.has_value());
            QCOMPARE(bins[static_cast<size_t>(c)].min, r->first);
            QCOMPARE(bins[static_cast<size_t>(c)].max, r->second);
        }
        QCOMPARE(samples, 1000);
        // The oldest sample is overwritten when full; time going back starts afresh.
        g.append(10.0, {1.0});
        QCOMPARE(g.size(), qsizetype(1000));
        QCOMPARE(g.firstTime(), 0.01);
        g.append(2.0, {1.0});
        QCOMPARE(g.size(), qsizetype(1));
        QCOMPARE(g.nearest(5.0).value(), qsizetype(0));
    }

    void rawTimeUsesOneOffset() {
        StreamClock clock;
        clock.setOdr(512.0);
        clock.noteBoardTime(10000); // t_ms of a preceding $ATT
        RawRecord r;
        r.seq = 5120 - 64; // the IMU started 0.125 s before the board clock's 10 s... in sample numbers
        r.n = 8;
        const double t0 = clock.rawTime(r);
        QVERIFY(std::abs(t0 - (10.0 + 4.0 / 512.0)) < 1e-9);
        clock.noteBoardTime(99999); // later t_ms values do not move the offset
        r.seq += 512;
        QVERIFY(std::abs(clock.rawTime(r) - (t0 + 1.0)) < 1e-9);
    }

    void drawingTenMinutesIsCheap() {
        StreamSeries s;
        for (int i = 0; i < 10 * 60 * 200; ++i) s.feed(attLine(i / 200.0, 20.0 * std::sin(i / 300.0) + (i % 7)));
        TimeView view;
        view.setWindow(600.0);
        view.setLatest(s.latestTime());
        PlotWidget plot(&view);
        plot.addTrace(&s.att, StreamSeries::Roll, QStringLiteral("roll"), PlotWidget::seriesColor(0));
        plot.addTrace(&s.att, StreamSeries::Pitch, QStringLiteral("pitch"), PlotWidget::seriesColor(1));
        plot.addTrace(&s.att, StreamSeries::Yaw, QStringLiteral("yaw"), PlotWidget::seriesColor(2));
        plot.resize(1400, 260);
        QImage image(plot.size(), QImage::Format_ARGB32);
        plot.render(&image); // warm-up
        QElapsedTimer clock;
        clock.start();
        const int frames = 10;
        for (int i = 0; i < frames; ++i) plot.render(&image);
        const double ms = static_cast<double>(clock.elapsed()) / frames;
        qInfo("one frame of 3 x 120000 samples: %.1f ms", ms);
        QVERIFY(ms < 120.0);
        // Something was drawn in the trace colour.
        int coloured = 0;
        for (int y = 0; y < image.height(); y += 2) {
            for (int x = 0; x < image.width(); x += 2) coloured += image.pixelColor(x, y) == PlotWidget::seriesColor(0);
        }
        QVERIFY(coloured > 50);
    }

    void boardViewWhereOpenGlExists() {
        BoardView view;
        view.resize(400, 300);
        view.show();
        QSignalSpy ready(&view, &GlScene::glReady);
        if (!ready.wait(3000) && !view.glInitialized()) QSKIP("no OpenGL context on this platform");
        if (!view.glAvailable()) QSKIP(qPrintable(QStringLiteral("OpenGL 3.3 not available: ") + view.glProblem()));
        // Turning the board 90 deg about Up moves the tip of its X axis.
        const QVector3D tip(2.45f, 0, 0);
        view.setAttitude({1, 0, 0, 0});
        const QPointF a = view.project(toQMatrix(Eigen::Matrix4d::Identity()).map(tip));
        view.setAttitude({std::sqrt(0.5), 0, 0, std::sqrt(0.5)});
        const QPointF b = view.project(QVector3D(0, 2.45f, 0));
        QVERIFY((a - b).manhattanLength() > 20.0);
        QImage frame = view.grabFramebuffer();
        QVERIFY(!frame.isNull());
        if (qEnvironmentVariableIsSet("INS10_SAVE_FRAME")) frame.save(qEnvironmentVariable("INS10_SAVE_FRAME"));
    }
};

QTEST_MAIN(TestPlots)
#include "tst_plots.moc"
