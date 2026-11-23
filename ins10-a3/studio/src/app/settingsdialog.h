// Application settings: interface language and the $ATT rate used for viewing.
#pragma once

#include <QDialog>

class QComboBox;
class QSpinBox;

namespace ins10 {

class SettingsDialog : public QDialog {
    Q_OBJECT
public:
    explicit SettingsDialog(QWidget *parent = nullptr);

    static int viewAttRate();

    void accept() override;

signals:
    void viewAttRateChanged(int hz);

private:
    QComboBox *m_language;
    QSpinBox *m_rate;
};

} // namespace ins10
