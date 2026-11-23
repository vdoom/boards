// All calibration fields as a table: view and edit, flag bits, declination; write to
// RAM, save to flash, discard changes, erase; export and import JSON and 'cal show'
// text (compatible with ins10.py cal export / import).
#pragma once

#include "device/boardmodel.h"

#include <QWidget>

class QCheckBox;
class QLabel;
class QPushButton;
class QTableWidget;

namespace ins10 {

class CalFieldsWidget : public QWidget {
    Q_OBJECT
public:
    explicit CalFieldsWidget(BoardModel *board, QWidget *parent = nullptr);

    // The table's contents (board values with the edits applied); fields that do not
    // parse are left out and reported in *errors.
    CalibrationSet edited(QStringList *errors = nullptr) const;
    // Fields that differ from what the board has.
    QStringList pendingChanges() const;
    void loadIntoTable(const CalibrationSet &set);

private:
    void rebuild();
    void refreshFromBoard();
    void updateMarks();
    void writeToRam(std::function<void(bool)> then = {});
    void saveToFlash();
    void discard();
    void erase();
    void importFile();
    void exportJson();
    void exportText();
    void setBusy(bool busy);
    void report(bool ok, const QString &message);
    int rowOf(const QString &name) const;

    BoardModel *m_board;
    QTableWidget *m_table;
    QList<QCheckBox *> m_flagBoxes;
    QCheckBox *m_magEnabled;
    QLabel *m_message;
    QList<QPushButton *> m_boardButtons;
    bool m_updating = false;
    bool m_busy = false;
};

} // namespace ins10
