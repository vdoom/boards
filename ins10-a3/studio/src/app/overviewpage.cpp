#include "app/overviewpage.h"

#include "app/boardview.h"
#include "app/compasswidget.h"
#include "app/uiutil.h"
#include "core/scene.h"

#include <QFrame>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QSettings>
#include <QTimer>
#include <QVBoxLayout>

namespace ins10 {

OverviewPage::OverviewPage(BoardModel *board, QWidget *parent)
    : QWidget(parent), m_board(board), m_view(new BoardView(this)), m_compass(new CompassWidget(this)),
      m_age(new QLabel(this)) {
    auto *left = new QVBoxLayout;
    left->addWidget(m_view, 1);
    left->addWidget(m_compass);
    auto *controls = new QHBoxLayout;
    auto *top = new QPushButton(this);
    auto *reset = new QPushButton(this);
    controls->addWidget(top);
    controls->addWidget(reset);
    controls->addStretch(1);
    auto *hint = new QLabel(this);
    hint->setEnabled(false);
    ui::translated(this, [top, reset, hint] {
        top->setText(tr("View from above"));
        reset->setText(tr("Default view"));
        hint->setText(tr("Drag to turn the view, wheel to zoom, double-click to reset."));
    });
    controls->addWidget(hint);
    left->addLayout(controls);

    m_tiles = new QGridLayout;
    m_tiles->setSpacing(6);
    auto *right = new QVBoxLayout;
    right->addLayout(m_tiles);
    m_roll = addTile(QT_TR_NOOP("Roll"), QT_TR_NOOP("Rotation about the body X axis, deg"), 0, 0);
    m_pitch = addTile(QT_TR_NOOP("Pitch"), QT_TR_NOOP("Rotation about the body Y axis, deg"), 0, 1);
    m_yaw = addTile(QT_TR_NOOP("Yaw"),
                    QT_TR_NOOP("Counter-clockwise from east seen from above, deg; zero when X points east"), 1, 0);
    m_heading = addTile(QT_TR_NOOP("Heading"), QT_TR_NOOP("Compass heading of the X axis, clockwise from north, deg"), 1, 1);
    m_tiltSigma = addTile(QT_TR_NOOP("Tilt sigma"), QT_TR_NOOP("Filter 1-sigma of the tilt, per axis, deg"), 2, 0);
    m_headingSigma = addTile(QT_TR_NOOP("Heading sigma"), QT_TR_NOOP("Filter 1-sigma of the heading, deg"), 2, 1);
    m_bias = addTile(QT_TR_NOOP("Gyro bias, deg/s"), QT_TR_NOOP("The filter's estimate of the gyro bias, X Y Z"), 3, 0, 2);
    m_temp = addTile(QT_TR_NOOP("Temperature"), QT_TR_NOOP("IMU die temperature, filtered"), 4, 0, 2);

    auto *flagsBox = new QGroupBox(this);
    auto *flagsLayout = new QVBoxLayout(flagsBox);
    flagsLayout->setSpacing(1);
    for (int bit = 0; bit < kStatusFlagCount; ++bit) {
        auto *label = new QLabel(this);
        flagsLayout->addWidget(label);
        m_flagRows.append(label);
    }
    ui::translated(this, [this, flagsBox] {
        flagsBox->setTitle(tr("Filter flags"));
        for (int bit = 0; bit < m_flagRows.size(); ++bit) m_flagRows[bit]->setToolTip(ui::flagDescription(bit));
    });
    right->addWidget(flagsBox);
    m_age->setEnabled(false);
    m_age->setWordWrap(true);
    right->addWidget(m_age);
    right->addStretch(1);

    auto *layout = new QHBoxLayout(this);
    layout->addLayout(left, 3);
    layout->addLayout(right, 1);

    // The board drawing could once be turned in quarter turns, while its axes were not
    // known; drop the setting that held the turn.
    QSettings().remove(QStringLiteral("overview/outlineTurns"));
    connect(top, &QPushButton::clicked, m_view, &GlScene::topView);
    connect(reset, &QPushButton::clicked, m_view, &GlScene::resetView);
    connect(board, &BoardModel::attitude, this, [this] { m_dirty = true; });
    connect(board, &BoardModel::connectionChanged, this, [this] { m_dirty = true; });
    // Redraw at most 30 times a second, whatever the $ATT rate.
    auto *timer = new QTimer(this);
    timer->setInterval(33);
    connect(timer, &QTimer::timeout, this, [this] {
        if (m_dirty && isVisible()) refresh();
    });
    timer->start();
    ui::translated(this, [this] { refresh(); });
}

QLabel *OverviewPage::addTile(const char *title, const char *tip, int row, int column, int columnSpan) {
    auto *frame = new QFrame(this);
    frame->setFrameShape(QFrame::StyledPanel);
    auto *layout = new QVBoxLayout(frame);
    layout->setContentsMargins(8, 4, 8, 6);
    auto *caption = new QLabel(frame);
    caption->setEnabled(false);
    ui::translated(frame, [frame, caption, title, tip] {
        caption->setText(tr(title));
        frame->setToolTip(tr(tip));
    });
    auto *value = new QLabel(QStringLiteral("-"), frame);
    QFont f = value->font();
    f.setPointSizeF(f.pointSizeF() * 1.7);
    f.setBold(true);
    value->setFont(f);
    value->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
    value->setTextInteractionFlags(Qt::TextSelectableByMouse);
    layout->addWidget(caption);
    layout->addWidget(value);
    m_tiles->addWidget(frame, row, column, 1, columnSpan);
    return value;
}

void OverviewPage::refresh() {
    m_dirty = false;
    const std::optional<AttRecord> &att = m_board->lastAtt();
    const QString deg = QStringLiteral("°");
    if (!att) {
        for (QLabel *l : {m_roll, m_pitch, m_yaw, m_heading, m_tiltSigma, m_headingSigma, m_bias, m_temp})
            l->setText(QStringLiteral("-"));
        m_compass->setHeading(std::nullopt, false);
        for (int bit = 0; bit < m_flagRows.size(); ++bit) {
            m_flagRows[bit]->setText(QStringLiteral("<span style='color:#999'>&#9675; %1</span>").arg(statusFlagNames()[bit]));
        }
        m_age->setText(m_board->isConnected() ? tr("No $ATT records: the attitude stream is off (rate 0).")
                                              : tr("Not connected."));
        return;
    }
    const SceneAttitude scene = sceneAttitude(*att);
    m_view->setAttitude(scene.q);
    m_roll->setText(ui::number(att->rollDeg, 2) + deg);
    m_pitch->setText(ui::number(att->pitchDeg, 2) + deg);
    m_yaw->setText(ui::number(att->yawDeg, 2) + deg);
    m_heading->setText(ui::number(att->headingDeg, 2) + deg);
    m_tiltSigma->setText(ui::number(att->tiltSigmaDeg, 2) + deg);
    m_headingSigma->setText(ui::number(att->headingSigmaDeg, 2) + deg);
    m_bias->setText(QStringLiteral("%1  %2  %3").arg(ui::number(att->biasDps[0], 3), ui::number(att->biasDps[1], 3),
                                                     ui::number(att->biasDps[2], 3)));
    m_temp->setText(ui::number(att->tempC, 2) + QStringLiteral(" °C"));
    const bool referenced = att->flags & 0x008;
    m_compass->setHeading(att->headingDeg, referenced);
    for (int bit = 0; bit < m_flagRows.size(); ++bit) {
        const bool on = att->flags & (1u << bit);
        m_flagRows[bit]->setText(on ? QStringLiteral("<span style='color:%1'>&#9679;</span> <b>%2</b>")
                                          .arg(ui::flagColor(bit).name(), statusFlagNames()[bit])
                                    : QStringLiteral("<span style='color:#999'>&#9675; %1</span>").arg(statusFlagNames()[bit]));
    }
    m_age->setText(tr("Board time %1 s").arg(ui::number(static_cast<double>(att->tMs) * 1e-3, 1)));
}

} // namespace ins10
