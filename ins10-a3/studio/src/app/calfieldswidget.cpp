#include "app/calfieldswidget.h"

#include "app/uiutil.h"

#include <QCheckBox>
#include <QCoreApplication>
#include <QFile>
#include <QFileDialog>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QMessageBox>
#include <QPushButton>
#include <QRegularExpression>
#include <QSaveFile>
#include <QTableWidget>
#include <QVBoxLayout>

namespace ins10 {

namespace {
enum Column { NameColumn, ValueColumn, UnitColumn, DescriptionColumn };

QString valuesText(const QList<double> &values) {
    QStringList parts;
    for (double v : values) parts.append(formatCalValue(v));
    return parts.join(u' ');
}

std::optional<QList<double>> parseValues(const QString &text) {
    QList<double> out;
    for (const QString &part : text.split(QRegularExpression(QStringLiteral("[\\s,;]+")), Qt::SkipEmptyParts)) {
        bool ok = false;
        const double v = part.toDouble(&ok);
        if (!ok) return std::nullopt;
        out.append(v);
    }
    return out;
}

const char *const kFlagLabels[kCalFlagCount] = {
    QT_TRANSLATE_NOOP("ins10::CalFieldsWidget", "gyro"), QT_TRANSLATE_NOOP("ins10::CalFieldsWidget", "accelerometer"),
    QT_TRANSLATE_NOOP("ins10::CalFieldsWidget", "magnetometer"), QT_TRANSLATE_NOOP("ins10::CalFieldsWidget", "gyro temperature"),
    QT_TRANSLATE_NOOP("ins10::CalFieldsWidget", "accelerometer temperature")};
} // namespace

CalFieldsWidget::CalFieldsWidget(BoardModel *board, QWidget *parent)
    : QWidget(parent), m_board(board), m_table(new QTableWidget(this)),
      m_magEnabled(new QCheckBox(this)), m_message(new QLabel(this)) {
    auto *layout = new QVBoxLayout(this);
    auto *intro = new QLabel(this);
    intro->setWordWrap(true);
    layout->addWidget(intro);

    auto *flags = new QHBoxLayout;
    auto *calibrated = new QLabel(this);
    flags->addWidget(calibrated);
    for (int bit = 0; bit < kCalFlagCount; ++bit) {
        auto *box = new QCheckBox(this);
        flags->addWidget(box);
        m_flagBoxes.append(box);
        connect(box, &QCheckBox::toggled, this, [this] {
            if (m_updating) return;
            quint32 value = 0;
            for (int b = 0; b < m_flagBoxes.size(); ++b) value |= m_flagBoxes[b]->isChecked() ? (1u << b) : 0u;
            m_updating = true;
            m_table->item(rowOf(QStringLiteral("flags")), ValueColumn)->setText(QString::number(value));
            m_updating = false;
            updateMarks();
        });
    }
    flags->addStretch(1);
    layout->addLayout(flags);

    m_table->setColumnCount(4);
    m_table->verticalHeader()->hide();
    m_table->horizontalHeader()->setSectionResizeMode(NameColumn, QHeaderView::ResizeToContents);
    m_table->horizontalHeader()->setSectionResizeMode(ValueColumn, QHeaderView::Stretch);
    m_table->horizontalHeader()->setSectionResizeMode(UnitColumn, QHeaderView::ResizeToContents);
    m_table->horizontalHeader()->setSectionResizeMode(DescriptionColumn, QHeaderView::Stretch);
    m_table->verticalHeader()->setDefaultSectionSize(fontMetrics().height() + 8);
    layout->addWidget(m_table, 1);
    layout->addWidget(m_magEnabled);

    auto *buttons = new QHBoxLayout;
    // text and tip untranslated (QT_TR_NOOP).
    auto addButton = [&](const char *text, const char *tip, auto slot, bool needsBoard) {
        auto *b = new QPushButton(this);
        ui::translated(b, [b, text, tip] {
            b->setText(tr(text));
            b->setToolTip(tr(tip));
        });
        connect(b, &QPushButton::clicked, this, slot);
        buttons->addWidget(b);
        if (needsBoard) m_boardButtons.append(b);
        return b;
    };
    addButton(QT_TR_NOOP("Read from the board"), QT_TR_NOOP("cal show"), &CalFieldsWidget::refreshFromBoard, true);
    addButton(QT_TR_NOOP("Write to RAM"), QT_TR_NOOP("Send the changed fields with 'cal set'"), [this] { writeToRam(); }, true);
    addButton(QT_TR_NOOP("Save to flash..."), QT_TR_NOOP("Write the changes, then 'cal save'"), &CalFieldsWidget::saveToFlash, true);
    addButton(QT_TR_NOOP("Discard changes"),
              QT_TR_NOOP("Back to the calibration found when connecting (or last saved): the board's RAM and this table"),
              &CalFieldsWidget::discard, true);
    addButton(QT_TR_NOOP("Erase..."), QT_TR_NOOP("cal erase: factory values, flash cleared, filter restarted"),
              &CalFieldsWidget::erase, true);
    buttons->addSpacing(16);
    addButton(QT_TR_NOOP("Import..."),
              QT_TR_NOOP("A JSON file from 'ins10.py cal export' or a saved 'cal show' listing; loaded into the table, not "
                         "written yet"),
              &CalFieldsWidget::importFile, false);
    addButton(QT_TR_NOOP("Export JSON..."), QT_TR_NOOP("The board's calibration as 'ins10.py cal export' writes it"),
              &CalFieldsWidget::exportJson, true);
    addButton(QT_TR_NOOP("Export text..."), QT_TR_NOOP("The board's calibration as 'cal show' lines"),
              &CalFieldsWidget::exportText, true);
    buttons->addStretch(1);
    layout->addLayout(buttons);
    m_message->setWordWrap(true);
    layout->addWidget(m_message);

    rebuild();
    ui::translated(this, [this, intro, calibrated] {
        m_magEnabled->setText(tr("Use the magnetometer ('set mag'; stored with the next save)"));
        intro->setText(tr("Every field of the board's calibration. Edit values (separated by spaces) and write them to "
                          "RAM: the filter uses them at once. Nothing reaches flash until you save. Changed values are "
                          "highlighted."));
        calibrated->setText(tr("Calibrated:"));
        for (int bit = 0; bit < m_flagBoxes.size(); ++bit) {
            m_flagBoxes[bit]->setText(QCoreApplication::translate("ins10::CalFieldsWidget", kFlagLabels[bit]));
            m_flagBoxes[bit]->setToolTip(tr("Bit %1 (value %2) of the flags field").arg(bit).arg(1 << bit));
        }
        m_table->setHorizontalHeaderLabels({tr("Field"), tr("Values"), tr("Unit"), tr("Meaning")});
        m_updating = true;
        const auto &fields = calibrationFields();
        for (int r = 0; r < m_table->rowCount() && r < fields.size(); ++r) {
            m_table->item(r, DescriptionColumn)
                ->setText(QCoreApplication::translate("ins10::CalibrationFields", fields[r].description));
        }
        m_updating = false;
    });
    connect(m_table, &QTableWidget::itemChanged, this, [this](QTableWidgetItem *item) {
        if (m_updating || item->column() != ValueColumn) return;
        if (item->row() == rowOf(QStringLiteral("flags"))) {
            const auto v = parseValues(item->text());
            const quint32 value = v && v->size() == 1 && v->front() >= 0 && v->front() < 256 ? static_cast<quint32>(v->front()) : 0;
            m_updating = true;
            for (int b = 0; b < m_flagBoxes.size(); ++b) m_flagBoxes[b]->setChecked(value & (1u << b));
            m_updating = false;
        }
        updateMarks();
    });
    connect(board, &BoardModel::calibrationChanged, this, [this] {
        if (pendingChanges().isEmpty() || !isVisible()) loadIntoTable(m_board->calibration());
        updateMarks();
    });
    connect(board, &BoardModel::infoChanged, this, [this] {
        m_updating = true;
        m_magEnabled->setChecked(m_board->info().text(QStringLiteral("mag_enabled")) != QLatin1String("0"));
        m_updating = false;
    });
    connect(m_magEnabled, &QCheckBox::toggled, this, [this](bool on) {
        if (m_updating || !m_board->isConnected()) return;
        m_board->device()->command(QStringLiteral("set mag %1").arg(on ? 1 : 0), [this](const CommandResult &r) {
            report(r.ok, r.ok ? tr("Magnetometer use changed; save to flash to keep it after a restart.") : r.error);
            m_board->refreshInfo();
        });
    });
    connect(board, &BoardModel::connectionChanged, this, [this] { setBusy(m_busy); });
    setBusy(false);
}

int CalFieldsWidget::rowOf(const QString &name) const {
    for (int r = 0; r < m_table->rowCount(); ++r) {
        if (m_table->item(r, NameColumn)->text() == name) return r;
    }
    return -1;
}

void CalFieldsWidget::rebuild() {
    m_updating = true;
    const auto &fields = calibrationFields();
    m_table->setRowCount(static_cast<int>(fields.size()));
    for (int r = 0; r < fields.size(); ++r) {
        const CalFieldDef &f = fields[r];
        auto *name = new QTableWidgetItem(QLatin1String(f.name));
        name->setFlags(name->flags() & ~Qt::ItemIsEditable);
        name->setFont(ui::monospaceFont());
        auto *value = new QTableWidgetItem;
        value->setFont(ui::monospaceFont());
        auto *unit = new QTableWidgetItem(QLatin1String(f.unit));
        unit->setFlags(unit->flags() & ~Qt::ItemIsEditable);
        auto *desc = new QTableWidgetItem(QCoreApplication::translate("ins10::CalibrationFields", f.description));
        desc->setFlags(desc->flags() & ~Qt::ItemIsEditable);
        m_table->setItem(r, NameColumn, name);
        m_table->setItem(r, ValueColumn, value);
        m_table->setItem(r, UnitColumn, unit);
        m_table->setItem(r, DescriptionColumn, desc);
    }
    m_updating = false;
    loadIntoTable(defaultCalibration());
}

void CalFieldsWidget::loadIntoTable(const CalibrationSet &set) {
    m_updating = true;
    for (auto it = set.cbegin(); it != set.cend(); ++it) {
        const int r = rowOf(it.key());
        if (r >= 0) m_table->item(r, ValueColumn)->setText(valuesText(it.value()));
    }
    const quint32 flags = calFlags(set);
    for (int b = 0; b < m_flagBoxes.size(); ++b) m_flagBoxes[b]->setChecked(flags & (1u << b));
    m_updating = false;
    updateMarks();
}

CalibrationSet CalFieldsWidget::edited(QStringList *errors) const {
    CalibrationSet out;
    for (int r = 0; r < m_table->rowCount(); ++r) {
        const QString name = m_table->item(r, NameColumn)->text();
        const auto values = parseValues(m_table->item(r, ValueColumn)->text());
        QString why;
        if (!values || !validateCalField(name, *values, &why)) {
            if (errors) errors->append(values ? why : tr("%1: not a list of numbers").arg(name));
            continue;
        }
        out.insert(name, *values);
    }
    return out;
}

QStringList CalFieldsWidget::pendingChanges() const {
    return changedCalFields(m_board->calibration(), edited());
}

void CalFieldsWidget::updateMarks() {
    const CalibrationSet board = m_board->calibration();
    const QColor changed = ui::levelColor(CheckLevel::Warning).lighter(165);
    const QColor bad = ui::levelColor(CheckLevel::Fail).lighter(170);
    m_updating = true;
    for (int r = 0; r < m_table->rowCount(); ++r) {
        QTableWidgetItem *item = m_table->item(r, ValueColumn);
        const QString name = m_table->item(r, NameColumn)->text();
        const auto values = parseValues(item->text());
        QBrush brush;
        if (!values || !validateCalField(name, *values)) brush = bad;
        else if (board.contains(name) && !sameCalValues(board.value(name), *values)) brush = changed;
        item->setBackground(brush);
        item->setForeground(brush.style() == Qt::NoBrush ? QBrush() : QBrush(Qt::black));
    }
    m_updating = false;
}

void CalFieldsWidget::setBusy(bool busy) {
    m_busy = busy;
    const bool connected = m_board->isConnected();
    for (QPushButton *b : m_boardButtons) b->setEnabled(connected && !busy);
    m_magEnabled->setEnabled(connected && !busy);
}

void CalFieldsWidget::report(bool ok, const QString &message) {
    m_message->setText(QStringLiteral("<span style='color:%1'>%2</span>")
                           .arg(ui::levelColor(ok ? CheckLevel::Ok : CheckLevel::Fail).name(), message.toHtmlEscaped()));
}

void CalFieldsWidget::refreshFromBoard() {
    setBusy(true);
    m_board->readCalibration([this](bool ok, const QString &error) {
        setBusy(false);
        if (ok) loadIntoTable(m_board->calibration());
        report(ok, ok ? tr("Read from the board.") : error);
    });
}

void CalFieldsWidget::writeToRam(std::function<void(bool)> then) {
    QStringList errors;
    const CalibrationSet all = edited(&errors);
    if (!errors.isEmpty()) {
        report(false, tr("Not written: %1").arg(errors.join(QStringLiteral("; "))));
        if (then) then(false);
        return;
    }
    CalibrationSet changes;
    for (const QString &name : changedCalFields(m_board->calibration(), all)) changes.insert(name, all.value(name));
    if (changes.isEmpty()) {
        report(true, tr("Nothing to write: the board already has these values."));
        if (then) then(true);
        return;
    }
    setBusy(true);
    m_board->writeCalibration(changes, 0, false, [this, then, count = changes.size()](bool ok, const QString &error) {
        setBusy(false);
        if (ok) loadIntoTable(m_board->calibration());
        report(ok, ok ? tr("%1 field(s) written to RAM.").arg(count) : error);
        if (then) then(ok);
    });
}

void CalFieldsWidget::saveToFlash() {
    const QStringList pending = pendingChanges();
    QString text = tr("Save the board's calibration to flash? It then survives a restart.");
    if (!pending.isEmpty()) text += u'\n' + tr("These changed fields are written first: %1.").arg(pending.join(QStringLiteral(", ")));
    if (QMessageBox::question(this, tr("Save to flash"), text) != QMessageBox::Yes) return;
    writeToRam([this](bool ok) {
        if (!ok) return;
        setBusy(true);
        m_board->saveCalibration([this](bool saved, const QString &error) {
            setBusy(false);
            if (saved) m_board->setBaseline(m_board->calibration());
            report(saved, saved ? tr("Saved to flash.") : error);
        });
    });
}

void CalFieldsWidget::discard() {
    if (m_board->baseline().isEmpty()) return;
    setBusy(true);
    m_board->restoreCalibration(m_board->baseline(), [this](bool ok, const QString &error) {
        setBusy(false);
        loadIntoTable(m_board->calibration());
        report(ok, ok ? tr("Back to the calibration found when connecting (or last saved).") : error);
    });
}

void CalFieldsWidget::erase() {
    const auto answer = QMessageBox::warning(
        this, tr("Erase the calibration"),
        tr("This erases the stored calibration, the learned thermal model and the settings in flash, puts the factory "
           "values in RAM and restarts the filter from initial alignment. It cannot be undone (export the calibration "
           "first if you may want it back).\n\nErase?"),
        QMessageBox::Yes | QMessageBox::Cancel, QMessageBox::Cancel);
    if (answer != QMessageBox::Yes) return;
    setBusy(true);
    m_board->device()->command(QStringLiteral("cal erase"), [this](const CommandResult &r) {
        m_board->readCalibration([this, r](bool, const QString &) {
            setBusy(false);
            if (r.ok) m_board->setBaseline(m_board->calibration());
            loadIntoTable(m_board->calibration());
            m_board->refreshInfo();
            report(r.ok, r.ok ? tr("Erased.") : r.error);
        });
    });
}

void CalFieldsWidget::importFile() {
    const QString path = QFileDialog::getOpenFileName(this, tr("Import a calibration"), QString(),
                                                      tr("Calibration (*.json *.txt *.cal);;All files (*)"));
    if (path.isEmpty()) return;
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly)) {
        report(false, file.errorString());
        return;
    }
    QString error;
    const auto set = parseCalibrationFile(file.readAll(), &error);
    if (!set) {
        report(false, tr("Not imported: %1").arg(error));
        return;
    }
    loadIntoTable(*set);
    report(true, tr("%1 field(s) loaded into the table; write them to RAM or save to flash to apply.").arg(set->size()));
}

void CalFieldsWidget::exportJson() {
    const QString path = QFileDialog::getSaveFileName(this, tr("Export the calibration"), QStringLiteral("calibration.json"),
                                                      tr("JSON (*.json)"));
    if (path.isEmpty()) return;
    QSaveFile file(path);
    if (!file.open(QIODevice::WriteOnly) || file.write(calibrationToJson(m_board->calibration())) < 0 || !file.commit()) {
        report(false, file.errorString());
        return;
    }
    report(true, tr("Written to %1").arg(path));
}

void CalFieldsWidget::exportText() {
    const QString path = QFileDialog::getSaveFileName(this, tr("Export the calibration"), QStringLiteral("calibration.txt"),
                                                      tr("Text (*.txt)"));
    if (path.isEmpty()) return;
    QSaveFile file(path);
    const QByteArray text = (calShowLines(m_board->calibration()).join(u'\n') + u'\n').toUtf8();
    if (!file.open(QIODevice::WriteOnly) || file.write(text) < 0 || !file.commit()) {
        report(false, file.errorString());
        return;
    }
    report(true, tr("Written to %1").arg(path));
}

} // namespace ins10
