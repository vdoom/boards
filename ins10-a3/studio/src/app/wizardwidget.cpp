#include "app/wizardwidget.h"

#include "app/language.h"
#include "app/uiutil.h"

#include <QFileDialog>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLocale>
#include <QMessageBox>
#include <QProgressBar>
#include <QPushButton>
#include <QSplitter>
#include <QTextBrowser>
#include <QVBoxLayout>

namespace ins10 {

QString htmlNumbers(const Eigen::Vector3d &v, int decimals, double factor) {
    return QStringLiteral("%1 &nbsp; %2 &nbsp; %3")
        .arg(ui::number(v[0] * factor, decimals), ui::number(v[1] * factor, decimals), ui::number(v[2] * factor, decimals));
}

QString htmlMatrix(const Eigen::Matrix3d &m, int decimals) {
    QString out = QStringLiteral("<table cellspacing=0 cellpadding=1 style='font-family:monospace'>");
    for (int r = 0; r < 3; ++r) {
        out += QStringLiteral("<tr>");
        for (int c = 0; c < 3; ++c) out += QStringLiteral("<td align=right>&nbsp;%1&nbsp;</td>").arg(ui::number(m(r, c), decimals));
        out += QStringLiteral("</tr>");
    }
    return out + QStringLiteral("</table>");
}

WizardWidget::WizardWidget(BoardModel *board, calib::Procedure procedure, QWidget *parent)
    : QWidget(parent), m_board(board), m_run(new CalibrationRun(board, procedure, this)) {}

bool WizardWidget::isBusy() const {
    const auto s = m_run->state();
    return s == CalibrationRun::State::Starting || s == CalibrationRun::State::Recording;
}

void WizardWidget::setBlocked(bool blocked) {
    m_blocked = blocked;
    updateControls();
}

void WizardWidget::build() {
    auto *layout = new QVBoxLayout(this);
    auto *intro = new QLabel(this);
    intro->setWordWrap(true);
    intro->setTextFormat(Qt::RichText);
    layout->addWidget(intro);

    auto *row = new QHBoxLayout;
    m_start = new QPushButton(this);
    m_stop = new QPushButton(this);
    m_load = new QPushButton(this);
    m_save = new QPushButton(this);
    m_status = new QLabel(this);
    row->addWidget(m_start);
    row->addWidget(m_stop);
    row->addSpacing(12);
    row->addWidget(m_load);
    row->addWidget(m_save);
    row->addSpacing(12);
    row->addWidget(m_status, 1);
    layout->addLayout(row);

    auto *splitter = new QSplitter(Qt::Vertical, this);
    auto *top = new QWidget(splitter);
    auto *topLayout = new QHBoxLayout(top);
    topLayout->setContentsMargins(0, 0, 0, 0);
    topLayout->addWidget(createLiveView(), 3);
    auto *side = new QVBoxLayout;
    side->addWidget(createFigures());
    m_extra = new QVBoxLayout;
    side->addLayout(m_extra);
    side->addStretch(1);
    topLayout->addLayout(side, 1);
    splitter->addWidget(top);

    auto *bottom = new QWidget(splitter);
    auto *bottomLayout = new QVBoxLayout(bottom);
    bottomLayout->setContentsMargins(0, 0, 0, 0);
    auto *computeRow = new QHBoxLayout;
    m_compute = new QPushButton(bottom);
    m_cancelCompute = new QPushButton(bottom);
    m_busy = new QProgressBar(bottom);
    m_busy->setRange(0, 0);
    m_busy->setMaximumWidth(160);
    m_busy->setVisible(false);
    computeRow->addWidget(m_compute);
    computeRow->addWidget(m_cancelCompute);
    computeRow->addWidget(m_busy);
    computeRow->addStretch(1);
    m_try = new QPushButton(bottom);
    m_flash = new QPushButton(bottom);
    m_revert = new QPushButton(bottom);
    computeRow->addWidget(m_try);
    computeRow->addWidget(m_flash);
    computeRow->addWidget(m_revert);
    bottomLayout->addLayout(computeRow);
    m_report = new QTextBrowser(bottom);
    m_report->setOpenLinks(false);
    bottomLayout->addWidget(m_report, 1);
    splitter->addWidget(bottom);
    splitter->setStretchFactor(0, 3);
    splitter->setStretchFactor(1, 2);
    // The live view is what the user steers by while recording; without explicit sizes
    // the splitter gave half of the height to the (then empty) report.
    splitter->setSizes({640, 360});
    layout->addWidget(splitter, 1);

    connect(m_start, &QPushButton::clicked, m_run, &CalibrationRun::start);
    connect(m_stop, &QPushButton::clicked, this, [this] {
        m_run->stop();
        if (m_run->state() == CalibrationRun::State::Recorded) m_run->compute();
    });
    connect(m_compute, &QPushButton::clicked, m_run, &CalibrationRun::compute);
    connect(m_cancelCompute, &QPushButton::clicked, m_run, &CalibrationRun::cancelCompute);
    connect(m_try, &QPushButton::clicked, m_run, &CalibrationRun::tryOnBoard);
    connect(m_flash, &QPushButton::clicked, this, &WizardWidget::confirmSave);
    connect(m_revert, &QPushButton::clicked, m_run, &CalibrationRun::revert);
    connect(m_load, &QPushButton::clicked, this, &WizardWidget::loadFile);
    connect(m_save, &QPushButton::clicked, this, &WizardWidget::saveFile);
    connect(m_run, &CalibrationRun::stateChanged, this, [this](CalibrationRun::State state) {
        if (state == CalibrationRun::State::Recording) {
            m_message.clear();
            reset();
            m_report->clear();
        }
        updateControls();
        emit busyChanged(isBusy());
    });
    connect(m_run, &CalibrationRun::progress, this, [this] {
        updateLive();
        updateControls();
    });
    connect(m_run, &CalibrationRun::computed, this, &WizardWidget::onComputed);
    connect(m_run, &CalibrationRun::written, this, &WizardWidget::onWritten);
    connect(m_run, &CalibrationRun::problem, this, [this](const QString &message) {
        m_message = QStringLiteral("<p style='color:%1'>%2</p>").arg(ui::levelColor(CheckLevel::Fail).name(), message.toHtmlEscaped());
        m_report->setHtml(m_message + reportHtml());
        updateControls();
    });
    connect(m_board, &BoardModel::connectionChanged, this, &WizardWidget::updateControls);
    ui::translated(this, [this, intro] {
        intro->setText(instructions());
        m_start->setText(tr("Start recording"));
        m_stop->setText(tr("Stop and compute"));
        m_load->setText(tr("Compute from a file..."));
        m_save->setText(tr("Save the recording..."));
        m_load->setToolTip(tr("A log written by this application or by 'ins10.py log' / --save-log. No board needed."));
        m_save->setToolTip(tr("Protocol text as 'ins10.py log' writes it: 'info' lines, the calibration in force, then "
                              "$RAW and $MAG. ins10.py --from-log and ins10_replay read it."));
        m_compute->setText(tr("Compute again"));
        m_cancelCompute->setText(tr("Cancel computation"));
        m_try->setText(tr("Try on the board"));
        m_try->setToolTip(tr("Write the result to the board's RAM. The filter uses it at once (see the Overview); "
                             "nothing is stored until you save."));
        m_flash->setText(tr("Save to flash..."));
        m_revert->setText(tr("Cancel: restore the previous calibration"));
        m_revert->setToolTip(tr("Send back the calibration read when the recording started (RAM only)."));
        updateControls();
    });
    // Progress, figures and report of the run so far, in the new language. (Messages
    // already given stay as they were.)
    connect(language::notifier(), &language::Notifier::changed, this, [this] {
        const Recording &rec = m_run->recording();
        if (rec.imuRows() + rec.magRows() > 0) updateLive();
        else reset();
        if (m_run->hasResult()) {
            showResult();
            m_report->setHtml(m_message + reportHtml());
        }
    });
}

void WizardWidget::updateControls() {
    using S = CalibrationRun::State;
    const S s = m_run->state();
    const bool connected = m_board->isConnected();
    const bool recording = s == S::Recording || s == S::Starting;
    const bool computing = s == S::Computing;
    const bool writing = s == S::Writing;
    const bool result = m_run->hasResult() && m_run->result().ok;
    const bool writable = result && m_run->result().writable && connected && !writing;
    m_start->setEnabled(connected && !recording && !computing && !writing && !m_blocked);
    m_stop->setEnabled(recording);
    m_load->setEnabled(!recording && !computing && !writing);
    m_save->setEnabled(!recording && !m_run->lines().isEmpty());
    m_compute->setEnabled((s == S::Recorded || s == S::Computed) && m_run->recording().imuRows() + m_run->recording().magRows() > 0);
    m_cancelCompute->setVisible(computing);
    m_busy->setVisible(computing || writing);
    m_try->setEnabled(writable && !m_run->tried());
    m_flash->setEnabled(writable && !m_run->saved());
    m_revert->setEnabled(connected && m_run->hasSnapshot() && m_run->tried() && !m_run->saved() && !writing);
    QString status;
    switch (s) {
    case S::Idle: status = connected ? tr("Ready.") : tr("Connect a board to record, or compute from a file."); break;
    case S::Starting: status = tr("Preparing the board..."); break;
    case S::Recording:
        status = tr("Recording: %1 s, %2 IMU rows, %3 magnetometer rows")
                     .arg(ui::number(m_run->recordedSeconds(), 0))
                     .arg(m_run->recording().imuRows())
                     .arg(m_run->recording().magRows());
        break;
    case S::Recorded: status = tr("Recorded %1 s.").arg(ui::number(m_run->recordedSeconds(), 0)); break;
    case S::Computing: status = tr("Computing..."); break;
    case S::Computed: status = m_run->isOffline() ? tr("Computed from a file.") : tr("Computed."); break;
    case S::Writing: status = tr("Writing to the board..."); break;
    }
    m_status->setText(status);
}

QString WizardWidget::reportHtml() const {
    if (!m_run->hasResult()) return {};
    const calib::WorkflowResult &r = m_run->result();
    QString html;
    const QString fail = ui::levelColor(CheckLevel::Fail).name(), warn = ui::levelColor(CheckLevel::Warning).name();
    if (!r.ok) return QStringLiteral("<p style='color:%1'><b>%2</b> %3</p>").arg(fail, tr("Not calibrated:"), r.error.toHtmlEscaped());
    html += resultHtml();
    if (!r.warnings.isEmpty()) {
        html += QStringLiteral("<p><b>%1</b><ul>").arg(tr("Warnings"));
        for (const calib::Warning &w : r.warnings) html += QStringLiteral("<li style='color:%1'>%2</li>").arg(warn, w.text.toHtmlEscaped());
        html += QStringLiteral("</ul></p>");
    }
    if (!r.writable) {
        html += QStringLiteral("<p style='color:%1'><b>%2</b></p>").arg(fail, r.refusal.toHtmlEscaped());
        return html;
    }
    html += QStringLiteral("<p><b>%1</b> (%2)</p><pre>").arg(tr("Fields to write"), tr("flag bits %1 are added").arg(r.setFlags));
    html += calShowLines(r.fields).join(u'\n').toHtmlEscaped();
    html += QStringLiteral("</pre>");
    return html;
}

void WizardWidget::onComputed() {
    showResult();
    m_report->setHtml(m_message + reportHtml());
    updateControls();
}

void WizardWidget::onWritten(bool ok, const QString &message) {
    const QString color = ui::levelColor(ok ? CheckLevel::Ok : CheckLevel::Fail).name();
    m_message = QStringLiteral("<p style='color:%1'><b>%2</b></p>").arg(color, message.toHtmlEscaped());
    m_report->setHtml(m_message + reportHtml());
    updateControls();
}

void WizardWidget::confirmSave() {
    const calib::WorkflowResult &r = m_run->result();
    QString text = tr("Write these fields to the board and save them to flash?\n\n%1\n\nThe flag bits %2 are added to the "
                      "flag word. The $ATT rate stored as the power-up rate stays the board's own.")
                       .arg(calShowLines(r.fields).join(u'\n'))
                       .arg(r.setFlags);
    if (QMessageBox::question(this, tr("Save to flash"), text) != QMessageBox::Yes) return;
    m_run->saveToFlash();
}

void WizardWidget::loadFile() {
    const QString path = QFileDialog::getOpenFileName(this, tr("Recording to compute from"), QString(),
                                                      tr("Logs (*.log *.txt);;All files (*)"));
    if (path.isEmpty()) return;
    QString error;
    if (!m_run->loadRecording(path, &error)) {
        QMessageBox::warning(this, tr("Recording"), tr("Could not use %1: %2").arg(path, error));
        return;
    }
    reset();
    updateLive();
    m_run->compute();
}

void WizardWidget::saveFile() {
    const QString path = QFileDialog::getSaveFileName(this, tr("Save the recording"), QStringLiteral("calibration.log"),
                                                      tr("Logs (*.log)"));
    if (path.isEmpty()) return;
    QString error;
    if (!m_run->saveRecording(path, &error)) QMessageBox::warning(this, tr("Recording"), tr("Could not write: %1").arg(error));
}

} // namespace ins10
