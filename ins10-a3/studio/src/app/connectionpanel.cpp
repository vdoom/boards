#include "app/connectionpanel.h"

#include "app/uiutil.h"
#include "device/ports.h"

#include <QAbstractItemView>
#include <QComboBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QStyle>
#include <QTimer>
#include <QToolButton>

namespace ins10 {

namespace {
// Short labels for the flag indicators; the full names are in PROTOCOL.md.
const char *const kFlagLabels[kStatusFlagCount] = {"ALN", "CRS", "STL", "HDG", "ACC", "MAG", "UNC", "SAT", "THR", "REC", "STA"};

QFrame *separator(QWidget *parent) {
    auto *line = new QFrame(parent);
    line->setFrameShape(QFrame::VLine);
    line->setFrameShadow(QFrame::Sunken);
    return line;
}
} // namespace

ConnectionPanel::ConnectionPanel(BoardModel *board, QWidget *parent)
    : QWidget(parent), m_board(board), m_ports(new QComboBox(this)), m_refresh(new QToolButton(this)),
      m_connect(new QPushButton(this)), m_state(new QLabel(this)), m_identity(new QLabel(this)),
      m_damaged(new QLabel(this)), m_lost(new QLabel(this)), m_portTimer(new QTimer(this)) {
    auto *layout = new QHBoxLayout(this);
    layout->setContentsMargins(6, 3, 6, 3);
    layout->setSpacing(6);

    m_ports->setMinimumContentsLength(18);
    m_ports->setSizeAdjustPolicy(QComboBox::AdjustToMinimumContentsLengthWithIcon);
    m_refresh->setIcon(style()->standardIcon(QStyle::SP_BrowserReload));
    layout->addWidget(m_ports);
    layout->addWidget(m_refresh);
    layout->addWidget(m_connect);
    layout->addWidget(separator(this));
    m_state->setMinimumWidth(150);
    layout->addWidget(m_state);
    layout->addWidget(separator(this));
    layout->addWidget(m_identity);
    layout->addWidget(separator(this));

    for (int bit = 0; bit < kStatusFlagCount; ++bit) {
        auto *label = new QLabel(QString::fromLatin1(kFlagLabels[bit]), this);
        label->setAlignment(Qt::AlignCenter);
        label->setMinimumWidth(34);
        layout->addWidget(label);
        m_flags.append(label);
    }
    layout->addWidget(separator(this));
    layout->addWidget(m_damaged);
    layout->addWidget(m_lost);
    layout->addStretch(1);

    connect(m_refresh, &QToolButton::clicked, this, &ConnectionPanel::refreshPorts);
    connect(m_connect, &QPushButton::clicked, this, &ConnectionPanel::toggleConnection);
    connect(board, &BoardModel::connectionChanged, this, &ConnectionPanel::updateState);
    connect(board, &BoardModel::infoChanged, this, &ConnectionPanel::updateIdentity);
    connect(board, &BoardModel::attitude, this, [this](const AttRecord &r) { updateFlags(r.flags, true); });
    connect(board, &BoardModel::statusUpdated, this, [this] {
        // Without $ATT (rate 0) the flags still come from 'status'.
        if (!m_board->lastAtt() && m_board->status().flags.valid) updateFlags(m_board->status().flags.value, true);
    });
    connect(board->device(), &DeviceController::countersChanged, this, &ConnectionPanel::updateCounters);
    // New boards appear while nothing is connected.
    m_portTimer->setInterval(2000);
    connect(m_portTimer, &QTimer::timeout, this, [this] {
        if (m_board->device()->state() == ConnectionState::Disconnected && !m_ports->view()->isVisible()) refreshPorts();
    });
    m_portTimer->start();

    refreshPorts();
    updateState(ConnectionState::Disconnected, QString());
    updateFlags(0, false);
    updateCounters(SessionCounters());
    ui::translated(this, [this] { retranslate(); });
}

void ConnectionPanel::retranslate() {
    m_ports->setToolTip(tr("Serial port. Raspberry Pi Pico boards (USB vendor 0x2E8A) are listed first."));
    if (const int demo = m_ports->findData(kDemoPortName); demo >= 0) m_ports->setItemText(demo, tr("Simulated board (demo)"));
    m_refresh->setToolTip(tr("Look for serial ports again"));
    for (int bit = 0; bit < m_flags.size(); ++bit) {
        m_flags[bit]->setToolTip(QStringLiteral("<b>%1</b> (0x%2)<br>%3")
                                     .arg(statusFlagNames()[bit])
                                     .arg(1u << bit, 3, 16, QLatin1Char('0'))
                                     .arg(ui::flagDescription(bit)));
    }
    m_damaged->setToolTip(tr("Lines that arrived truncated, run together or unrecognised. The firmware drops "
                             "output when the PC does not read fast enough."));
    m_lost->setToolTip(tr("IMU samples missing between $RAW lines (from the sample numbers)."));
    // What the board said last, in the new language.
    DeviceController *device = m_board->device();
    updateState(device->state(), device->stateDetail());
    updateCounters(device->counters());
}

void ConnectionPanel::refreshPorts() {
    const QString current = m_ports->currentData().toString();
    const QList<PortInfo> ports = availablePorts();
    QStringList names;
    for (const PortInfo &p : ports) names.append(p.name);
    names.append(kDemoPortName);
    QStringList existing;
    for (int i = 0; i < m_ports->count(); ++i) existing.append(m_ports->itemData(i).toString());
    if (existing == names) return;

    m_ports->clear();
    int pico = -1;
    for (const PortInfo &p : ports) {
        m_ports->addItem(p.isPico ? ui::dotIcon(QColor(0x2e, 0x9d, 0x4f)) : QIcon(), p.description, p.name);
        if (p.isPico && pico < 0) pico = m_ports->count() - 1;
    }
    m_ports->addItem(tr("Simulated board (demo)"), kDemoPortName);
    int index = m_ports->findData(current);
    if (index < 0) index = pico >= 0 ? pico : (ports.isEmpty() ? m_ports->count() - 1 : 0);
    m_ports->setCurrentIndex(index);
}

void ConnectionPanel::connectDemo() {
    const int index = m_ports->findData(kDemoPortName);
    if (index >= 0) m_ports->setCurrentIndex(index);
    m_board->device()->connectTo(kDemoPortName);
}

void ConnectionPanel::toggleConnection() {
    DeviceController *device = m_board->device();
    const ConnectionState state = device->state();
    if (state == ConnectionState::Disconnected || state == ConnectionState::Lost) {
        const QString port = m_ports->currentData().toString();
        if (!port.isEmpty()) device->connectTo(port);
    } else {
        device->disconnectFromBoard();
    }
}

void ConnectionPanel::updateState(ConnectionState state, const QString &detail) {
    const bool idle = state == ConnectionState::Disconnected || state == ConnectionState::Lost;
    m_connect->setText(idle ? tr("Connect") : tr("Disconnect"));
    m_connect->setEnabled(state != ConnectionState::Closing);
    m_ports->setEnabled(idle);
    m_refresh->setEnabled(idle);
    CheckLevel level = CheckLevel::Pending;
    if (state == ConnectionState::Connected) level = CheckLevel::Ok;
    else if (state == ConnectionState::NotAhrs || state == ConnectionState::Lost) level = CheckLevel::Fail;
    else if (state != ConnectionState::Disconnected) level = CheckLevel::Warning;
    m_state->setText(QStringLiteral("<span style='color:%1'>&#9679;</span> %2")
                         .arg(ui::levelColor(level).name(), connectionStateText(state)));
    m_state->setToolTip(detail);
    if (idle) {
        updateFlags(0, false);
        m_identity->setText(tr("no board"));
    }
    if (state == ConnectionState::Connected) updateIdentity();
}

void ConnectionPanel::updateIdentity() {
    const InfoReport &info = m_board->info();
    if (!info.isAhrs()) return;
    const QString clock = info.clock();
    const QString clockText = clock == QLatin1String("external")
                                  ? tr("clock external")
                                  : QStringLiteral("<span style='color:%1'>%2</span>")
                                        .arg(ui::levelColor(CheckLevel::Warning).name(), tr("clock %1").arg(clock));
    m_identity->setText(tr("fw %1 · %2 · %3").arg(info.firmwareVersion(), info.board(), clockText));
    m_identity->setToolTip(info.lines.join(u'\n'));
}

void ConnectionPanel::updateFlags(quint32 flags, bool valid) {
    const bool dark = ui::darkPalette(this);
    for (int bit = 0; bit < m_flags.size(); ++bit) {
        const bool on = valid && (flags & (1u << bit));
        const QColor color = ui::flagColor(bit);
        if (on) {
            m_flags[bit]->setStyleSheet(QStringLiteral("QLabel{background:%1;color:white;border-radius:3px;padding:1px 3px;}")
                                            .arg(color.name()));
        } else {
            m_flags[bit]->setStyleSheet(QStringLiteral("QLabel{color:%1;border:1px solid %1;border-radius:3px;padding:0px 2px;}")
                                            .arg(dark ? QStringLiteral("#666") : QStringLiteral("#bbb")));
        }
    }
}

void ConnectionPanel::updateCounters(const SessionCounters &c) {
    m_damaged->setText(tr("damaged lines: %1").arg(c.damagedLines));
    m_lost->setText(tr("lost samples: %1").arg(c.lostSamples));
}

} // namespace ins10
