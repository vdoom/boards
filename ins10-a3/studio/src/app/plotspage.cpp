#include "app/plotspage.h"

#include "app/plotwidget.h"
#include "app/uiutil.h"

#include <QCheckBox>
#include <QComboBox>
#include <QDir>
#include <QFile>
#include <QFileDialog>
#include <QFileInfo>
#include <QHBoxLayout>
#include <QLabel>
#include <QMessageBox>
#include <QPushButton>
#include <QScrollArea>
#include <QSettings>
#include <QTextStream>
#include <QTimer>
#include <QVBoxLayout>

#include <set>

namespace ins10 {

namespace {
const QString kOwner = QStringLiteral("plots");
}

PlotsPage::PlotsPage(BoardModel *board, QWidget *parent)
    : QWidget(parent), m_board(board), m_view(new TimeView(this)), m_window(new QComboBox(this)),
      m_rawDiv(new QComboBox(this)), m_pause(new QPushButton(this)), m_repaint(new QTimer(this)),
      m_plotArea(new QWidget(this)) {
    auto *layout = new QVBoxLayout(this);
    auto *bar = new QHBoxLayout;
    m_pause->setCheckable(true);
    bar->addWidget(m_pause);
    auto *windowLabel = new QLabel(this);
    bar->addWidget(windowLabel);
    for (double seconds : {10.0, 30.0, 60.0, 120.0, 300.0, 600.0}) m_window->addItem(QString(), seconds);
    bar->addWidget(m_window);
    bar->addSpacing(12);
    auto *rateLabel = new QLabel(this);
    bar->addWidget(rateLabel);
    for (int n : {1, 2, 4, 8, 16}) m_rawDiv->addItem(QString(), n);
    bar->addWidget(m_rawDiv);
    bar->addSpacing(12);
    auto *exportButton = new QPushButton(this);
    bar->addWidget(exportButton);
    bar->addStretch(1);
    layout->addLayout(bar);

    auto *toggles = new QHBoxLayout;
    auto *showLabel = new QLabel(this);
    toggles->addWidget(showLabel);
    layout->addLayout(toggles);
    auto *plotsLayout = new QVBoxLayout(m_plotArea);
    plotsLayout->setContentsMargins(0, 0, 0, 0);
    auto *scroll = new QScrollArea(this);
    scroll->setWidgetResizable(true);
    scroll->setWidget(m_plotArea);
    scroll->setFrameShape(QFrame::NoFrame);
    layout->addWidget(scroll, 1);

    using N = Plot::Needs;
    using S = StreamSeries;
    addPlot(QStringLiteral("angles"), QT_TR_NOOP("Roll, pitch, yaw"), QT_TR_NOOP("deg"), N::Att, &m_series.att,
            {S::Roll, S::Pitch, S::Yaw}, true);
    addPlot(QStringLiteral("heading"), QT_TR_NOOP("Heading"), QT_TR_NOOP("deg"), N::Att, &m_series.att, {S::Heading}, true);
    addPlot(QStringLiteral("bias"), QT_TR_NOOP("Gyro bias estimate"), QT_TR_NOOP("deg/s"), N::Att, &m_series.att,
            {S::BiasX, S::BiasY, S::BiasZ}, true);
    addPlot(QStringLiteral("sigma"), QT_TR_NOOP("Filter sigma"), QT_TR_NOOP("deg"), N::Att, &m_series.att,
            {S::TiltSigma, S::HeadingSigma}, false);
    addPlot(QStringLiteral("temp"), QT_TR_NOOP("Temperature"), QT_TR_NOOP("C"), N::Att, &m_series.att, {S::AttTemp}, false);
    addPlot(QStringLiteral("gyro"), QT_TR_NOOP("Raw gyro"), QT_TR_NOOP("deg/s"), N::Raw, &m_series.raw,
            {S::GyroX, S::GyroY, S::GyroZ}, false);
    addPlot(QStringLiteral("accel"), QT_TR_NOOP("Raw accelerometer"), QT_TR_NOOP("g"), N::Raw, &m_series.raw,
            {S::AccelX, S::AccelY, S::AccelZ}, false);
    addPlot(QStringLiteral("mag"), QT_TR_NOOP("Magnetic field magnitude (raw)"), QT_TR_NOOP("uT"), N::Mag, &m_series.mag,
            {S::MagNorm}, false);
    addPlot(QStringLiteral("baro"), QT_TR_NOOP("Barometric altitude"), QT_TR_NOOP("m"), N::Baro, &m_series.baro,
            {S::Altitude}, false);
    for (const Plot &p : m_plots) toggles->addWidget(p.toggle);
    toggles->addStretch(1);
    plotsLayout->addStretch(1);

    ui::translated(this, [this, windowLabel, rateLabel, exportButton, showLabel] {
        m_pause->setText(tr("Pause"));
        m_pause->setToolTip(tr("Freeze the plots; drag to scroll back in time, wheel to zoom"));
        windowLabel->setText(tr("Window"));
        const QStringList windows = {tr("10 s"), tr("30 s"), tr("1 min"), tr("2 min"), tr("5 min"), tr("10 min")};
        for (int i = 0; i < m_window->count() && i < windows.size(); ++i) m_window->setItemText(i, windows[i]);
        rateLabel->setText(tr("Raw data rate"));
        for (int i = 0; i < m_rawDiv->count(); ++i) {
            const int n = m_rawDiv->itemData(i).toInt();
            m_rawDiv->setItemText(i, tr("%1 Hz (raw %2)").arg(512 / n).arg(n));
        }
        m_rawDiv->setToolTip(tr("'stream raw' divisor while a raw plot is shown: 512 Hz divided by this"));
        exportButton->setText(tr("Export visible to CSV..."));
        showLabel->setText(tr("Show:"));
        m_series.retranslate();
        for (const Plot &p : m_plots) {
            p.widget->setTitle(tr(p.title));
            p.widget->setUnit(tr(p.unit));
            p.widget->relabelTraces();
            p.toggle->setText(tr(p.title));
        }
    });

    QSettings settings;
    m_window->setCurrentIndex(std::clamp(settings.value(QStringLiteral("plots/window"), 1).toInt(), 0, m_window->count() - 1));
    m_rawDiv->setCurrentIndex(std::clamp(settings.value(QStringLiteral("plots/rawDiv"), 2).toInt(), 0, m_rawDiv->count() - 1));
    m_view->setWindow(m_window->currentData().toDouble());
    connect(m_window, &QComboBox::currentIndexChanged, this, [this](int i) {
        m_view->setWindow(m_window->itemData(i).toDouble());
        QSettings().setValue(QStringLiteral("plots/window"), i);
    });
    connect(m_rawDiv, &QComboBox::currentIndexChanged, this, [this](int i) {
        QSettings().setValue(QStringLiteral("plots/rawDiv"), i);
        updateStreams();
    });
    connect(m_pause, &QPushButton::toggled, m_view, &TimeView::setPaused);
    connect(m_view, &TimeView::changed, this, [this] {
        if (m_pause->isChecked() != m_view->paused()) m_pause->setChecked(m_view->paused());
    });
    connect(exportButton, &QPushButton::clicked, this, &PlotsPage::exportDialog);

    // Data is collected whether or not the page is shown, and the streams the ticked
    // plots need stay on while another page is open (see updateStreams()).
    connect(board->device(), &DeviceController::streamBatch, this, [this](const StreamBatch &batch) {
        for (const ParsedLine &line : batch) m_series.feed(line);
    });
    connect(board, &BoardModel::infoChanged, this, [this] { m_series.setInfo(m_board->info()); });
    connect(board, &BoardModel::connectionChanged, this, [this](ConnectionState state) {
        if (state == ConnectionState::Opening) {
            m_series.clear();
            m_view->reset();
        }
        if (state == ConnectionState::Connected) updateStreams();
    });
    m_repaint->setInterval(40);
    connect(m_repaint, &QTimer::timeout, this, [this] { m_view->setLatest(m_series.latestTime()); });
    updateStreams(); // in case the board is already connected
}

PlotsPage::~PlotsPage() = default;

void PlotsPage::addPlot(const QString &key, const char *title, const char *unit, Plot::Needs needs,
                        const TimeSeriesGroup *group, const QList<int> &channels, bool on) {
    auto *widget = new PlotWidget(m_view, m_plotArea);
    widget->setMinimumHeight(150);
    for (qsizetype i = 0; i < channels.size(); ++i) {
        widget->addTrace(group, channels[i], group->names()[channels[i]],
                         PlotWidget::seriesColor(channels.size() == 1 ? 2 + static_cast<int>(m_plots.size()) % 6 : static_cast<int>(i)));
    }
    auto *toggle = new QCheckBox(this);
    toggle->setChecked(QSettings().value(QStringLiteral("plots/show/") + key, on).toBool());
    widget->setVisible(toggle->isChecked());
    static_cast<QVBoxLayout *>(m_plotArea->layout())->addWidget(widget);
    connect(toggle, &QCheckBox::toggled, this, [this, widget, key](bool shown) {
        widget->setVisible(shown);
        QSettings().setValue(QStringLiteral("plots/show/") + key, shown);
        updateStreams();
    });
    m_plots.append({key, widget, toggle, needs, title, unit});
}

void PlotsPage::showEvent(QShowEvent *event) {
    QWidget::showEvent(event);
    m_repaint->start();
}

void PlotsPage::hideEvent(QHideEvent *event) {
    QWidget::hideEvent(event);
    m_repaint->stop();
}

// The streams follow the plot toggles only, not whether the page is shown. Switching
// them off while another page was open (to type 'status' in the console, say) left a
// hole in the magnetometer and barometer plots that looked like the board dropping
// data. Unticking a plot is what stops its stream.
void PlotsPage::updateStreams() {
    StreamRequest request;
    for (const Plot &p : m_plots) {
        if (!p.toggle->isChecked()) continue;
        if (p.needs == Plot::Needs::Raw) request.rawDiv = m_rawDiv->currentData().toInt();
        if (p.needs == Plot::Needs::Mag) request.mag = true;
        if (p.needs == Plot::Needs::Baro) request.baro = true;
    }
    m_board->device()->setStreamRequest(kOwner, request);
}

bool PlotsPage::exportVisible(const QString &basePath, QString *error) const {
    const double t0 = m_view->start(), t1 = m_view->end();
    std::set<const TimeSeriesGroup *> groups;
    for (const Plot &p : m_plots) {
        if (!p.toggle->isChecked()) continue;
        for (const PlotTrace &trace : p.widget->traces()) groups.insert(trace.group);
    }
    const QFileInfo base(basePath);
    const QString stem = base.dir().filePath(base.completeBaseName());
    const std::pair<const TimeSeriesGroup *, QString> names[] = {
        {&m_series.att, QStringLiteral("att")}, {&m_series.raw, QStringLiteral("raw")},
        {&m_series.mag, QStringLiteral("mag")}, {&m_series.baro, QStringLiteral("baro")}};
    for (const auto &[group, suffix] : names) {
        if (!groups.count(group)) continue;
        QFile file(stem + u'-' + suffix + QStringLiteral(".csv"));
        if (!file.open(QIODevice::WriteOnly | QIODevice::Truncate | QIODevice::Text)) {
            if (error) *error = file.errorString();
            return false;
        }
        QTextStream out(&file);
        out << "t_s";
        for (const QString &name : group->names()) out << ',' << QString(name).replace(u' ', u'_').remove(u'|');
        out << '\n';
        for (qsizetype i = group->lowerBound(t0); i < group->size() && group->time(i) <= t1; ++i) {
            out << QString::number(group->time(i), 'f', 4);
            for (int c = 0; c < group->channels(); ++c) out << ',' << QString::number(group->value(c, i), 'g', 7);
            out << '\n';
        }
    }
    return true;
}

void PlotsPage::exportDialog() {
    const QString path = QFileDialog::getSaveFileName(this, tr("Export visible data"), QStringLiteral("plots.csv"),
                                                      tr("CSV files (*.csv)"));
    if (path.isEmpty()) return;
    QString error;
    if (!exportVisible(path, &error)) QMessageBox::warning(this, tr("Export"), tr("Could not write: %1").arg(error));
}

} // namespace ins10
