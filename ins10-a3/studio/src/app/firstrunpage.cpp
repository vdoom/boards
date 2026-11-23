#include "app/firstrunpage.h"

#include "app/uiutil.h"

#include <QApplication>
#include <QClipboard>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QPushButton>
#include <QTreeWidget>
#include <QVBoxLayout>

namespace ins10 {

FirstRunPage::FirstRunPage(BoardModel *board, QWidget *parent)
    : QWidget(parent), m_board(board), m_checks(new QTreeWidget(this)), m_summary(new QLabel(this)) {
    auto *layout = new QVBoxLayout(this);
    auto *intro = new QLabel(this);
    intro->setWordWrap(true);
    layout->addWidget(intro);

    m_checks->setColumnCount(3);
    m_checks->setRootIsDecorated(true);
    m_checks->setWordWrap(true);
    m_checks->header()->setSectionResizeMode(0, QHeaderView::ResizeToContents);
    m_checks->header()->setSectionResizeMode(1, QHeaderView::ResizeToContents);
    m_checks->header()->setStretchLastSection(true);
    layout->addWidget(m_checks, 1);

    auto *row = new QHBoxLayout;
    m_summary->setWordWrap(true);
    row->addWidget(m_summary, 1);
    auto *again = new QPushButton(this);
    auto *copy = new QPushButton(this);
    row->addWidget(again);
    row->addWidget(copy);
    layout->addLayout(row);

    connect(again, &QPushButton::clicked, this, [this] { m_board->refreshInfo(); });
    connect(copy, &QPushButton::clicked, this, &FirstRunPage::copyReport);
    connect(board, &BoardModel::statusUpdated, this, &FirstRunPage::evaluate);
    connect(board, &BoardModel::infoChanged, this, &FirstRunPage::evaluate);
    connect(board, &BoardModel::connectionChanged, this, &FirstRunPage::evaluate);
    ui::translated(this, [this, intro, again, copy] {
        intro->setText(tr("<b>First run on the board.</b> These checks show whether the module is soldered and wired "
                          "correctly and whether its sensors, clock and data stream work. Lay the board still, "
                          "components up, and leave this page open for about half a minute. If something fails, the "
                          "details tell what to look at; the raw replies are in the Console page."));
        m_checks->setHeaderLabels({tr("Check"), tr("Result"), tr("Measured")});
        again->setText(tr("Read info again"));
        copy->setText(tr("Copy report"));
        copy->setToolTip(tr("Copy the checklist with the measured values as text"));
        evaluate();
    });
}

void FirstRunPage::showEvent(QShowEvent *event) {
    QWidget::showEvent(event);
    m_board->wantStatus(this, true);
}

void FirstRunPage::hideEvent(QHideEvent *event) {
    QWidget::hideEvent(event);
    m_board->wantStatus(this, false);
}

void FirstRunPage::evaluate() {
    const bool connected = m_board->isConnected();
    m_results = evaluateFirstRun(connected ? m_board->info() : InfoReport(), m_board->statusHistory());
    m_checks->clear();
    int ok = 0, warn = 0, fail = 0;
    for (const CheckResult &c : m_results) {
        auto *item = new QTreeWidgetItem(m_checks, {c.title, ui::levelText(c.level), c.detail});
        item->setIcon(0, ui::levelIcon(c.level));
        item->setForeground(1, ui::levelColor(c.level));
        if (!c.advice.isEmpty()) {
            auto *advice = new QTreeWidgetItem(item, {QString(), QString(), c.advice});
            advice->setFirstColumnSpanned(false);
            item->setExpanded(c.level == CheckLevel::Fail || c.level == CheckLevel::Warning);
        }
        ok += c.level == CheckLevel::Ok;
        warn += c.level == CheckLevel::Warning;
        fail += c.level == CheckLevel::Fail;
    }
    if (!connected) m_summary->setText(tr("Connect a board to run the checks."));
    else m_summary->setText(tr("%1 passed, %2 need attention, %3 failed, %4 still waiting.")
                                .arg(ok).arg(warn).arg(fail).arg(m_results.size() - ok - warn - fail));
}

void FirstRunPage::copyReport() {
    QStringList lines;
    lines.append(tr("INS10 first-run checks"));
    for (const QString &l : m_board->info().lines) lines.append(l);
    for (const CheckResult &c : m_results) {
        lines.append(QStringLiteral("[%1] %2: %3").arg(ui::levelText(c.level), c.title, c.detail));
        if (!c.advice.isEmpty()) lines.append(QStringLiteral("    ") + c.advice);
    }
    QApplication::clipboard()->setText(lines.join(u'\n'));
}

} // namespace ins10
