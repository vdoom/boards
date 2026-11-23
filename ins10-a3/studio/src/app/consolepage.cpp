#include "app/consolepage.h"

#include "app/uiutil.h"

#include <QCheckBox>
#include <QHBoxLayout>
#include <QKeyEvent>
#include <QLabel>
#include <QLineEdit>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QScrollBar>
#include <QSettings>
#include <QVBoxLayout>

namespace ins10 {

ConsolePage::ConsolePage(BoardModel *board, QWidget *parent)
    : QWidget(parent), m_board(board), m_log(new QPlainTextEdit(this)), m_input(new QLineEdit(this)),
      m_hideStreams(new QCheckBox(this)) {
    auto *layout = new QVBoxLayout(this);
    m_log->setReadOnly(true);
    m_log->setFont(ui::monospaceFont());
    m_log->setMaximumBlockCount(5000);
    m_log->setLineWrapMode(QPlainTextEdit::NoWrap);
    layout->addWidget(m_log, 1);

    auto *row = new QHBoxLayout;
    auto *prompt = new QLabel(QStringLiteral(">"), this);
    prompt->setFont(ui::monospaceFont());
    m_input->setFont(ui::monospaceFont());
    m_input->setMaxLength(kMaxCommandLength);
    m_input->installEventFilter(this);
    auto *sendButton = new QPushButton(this);
    auto *clear = new QPushButton(this);
    m_hideStreams->setChecked(QSettings().value(QStringLiteral("console/hideStreams"), true).toBool());
    row->addWidget(prompt);
    row->addWidget(m_input, 1);
    row->addWidget(sendButton);
    row->addWidget(m_hideStreams);
    row->addWidget(clear);
    layout->addLayout(row);

    auto *note = new QLabel(this);
    note->setWordWrap(true);
    note->setEnabled(false);
    layout->addWidget(note);
    ui::translated(this, [this, sendButton, clear, note] {
        m_hideStreams->setText(tr("Hide streams"));
        m_hideStreams->setToolTip(tr("Do not show $ATT, $RAW, $MAG and $BARO records"));
        m_input->setPlaceholderText(tr("Command, e.g. help, info, status, cal show"));
        sendButton->setText(tr("Send"));
        clear->setText(tr("Clear"));
        note->setText(tr("Commands typed here go to the board as they are. A 'stream att' rate set here becomes the "
                         "board's own rate, the one 'cal save' stores."));
    });

    connect(m_input, &QLineEdit::returnPressed, this, &ConsolePage::send);
    connect(sendButton, &QPushButton::clicked, this, &ConsolePage::send);
    connect(clear, &QPushButton::clicked, m_log, &QPlainTextEdit::clear);
    connect(m_hideStreams, &QCheckBox::toggled, this,
            [](bool on) { QSettings().setValue(QStringLiteral("console/hideStreams"), on); });
    connect(board->device(), &DeviceController::traffic, this, &ConsolePage::onTraffic);
    connect(board->device(), &DeviceController::commandFinished, this, [this](const CommandResult &r) {
        if (r.origin != CommandOrigin::User || r.ok) return;
        append(QStringLiteral("<span style='color:%1'>%2</span>")
                   .arg(ui::levelColor(CheckLevel::Fail).name(),
                        tr("%1: %2").arg(r.command, r.error).toHtmlEscaped()));
    });
    connect(board, &BoardModel::connectionChanged, this, [this](ConnectionState state, const QString &detail) {
        QString text = QStringLiteral("-- %1").arg(connectionStateText(state));
        if (!detail.isEmpty()) text += QStringLiteral(": ") + detail;
        append(QStringLiteral("<i>%1</i>").arg(text.toHtmlEscaped()));
    });
}

void ConsolePage::send() {
    const QString text = m_input->text().trimmed();
    if (text.isEmpty()) return;
    if (m_history.isEmpty() || m_history.last() != text) m_history.append(text);
    m_historyPos = -1;
    m_input->clear();
    m_board->device()->command(text, {}, CommandOrigin::User);
}

bool ConsolePage::eventFilter(QObject *watched, QEvent *event) {
    if (watched == m_input && event->type() == QEvent::KeyPress) {
        const auto *key = static_cast<QKeyEvent *>(event);
        if ((key->key() == Qt::Key_Up || key->key() == Qt::Key_Down) && !m_history.isEmpty()) {
            if (m_historyPos < 0) m_historyPos = static_cast<int>(m_history.size());
            m_historyPos += key->key() == Qt::Key_Up ? -1 : 1;
            m_historyPos = std::clamp(m_historyPos, 0, static_cast<int>(m_history.size()));
            m_input->setText(m_historyPos < m_history.size() ? m_history[m_historyPos] : QString());
            return true;
        }
    }
    return QWidget::eventFilter(watched, event);
}

void ConsolePage::onTraffic(const TrafficBatch &batch) {
    QStringList html;
    const bool hide = m_hideStreams->isChecked();
    const QString warn = ui::levelColor(CheckLevel::Warning).name();
    const QString fail = ui::levelColor(CheckLevel::Fail).name();
    for (const TrafficLine &line : batch) {
        const bool stream = line.kind == LineKind::Att || line.kind == LineKind::Raw || line.kind == LineKind::Mag ||
                            line.kind == LineKind::Baro;
        if (hide && stream) continue;
        const QString text = line.text.toHtmlEscaped();
        if (line.outgoing) html.append(QStringLiteral("<b>&gt; %1</b>").arg(text));
        else if (line.damaged) html.append(QStringLiteral("<span style='color:%1'>%2</span>").arg(warn, text));
        else if (line.kind == LineKind::Err) html.append(QStringLiteral("<span style='color:%1'>%2</span>").arg(fail, text));
        else html.append(text.isEmpty() ? QStringLiteral("&nbsp;") : text);
    }
    if (html.isEmpty()) return;
    // Keep the view where the user scrolled to, unless it was at the bottom.
    QScrollBar *bar = m_log->verticalScrollBar();
    const bool atBottom = bar->value() >= bar->maximum() - 2;
    for (const QString &h : html) m_log->appendHtml(h);
    if (atBottom) bar->setValue(bar->maximum());
}

void ConsolePage::append(const QString &html) { m_log->appendHtml(html); }

} // namespace ins10
