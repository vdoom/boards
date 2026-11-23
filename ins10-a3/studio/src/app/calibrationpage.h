// Calibration: the fields table and the three wizards. Only one wizard records at a
// time (they would fight over the stream settings).
#pragma once

#include "device/boardmodel.h"

#include <QWidget>

class QTabWidget;

namespace ins10 {

class CalFieldsWidget;
class ImuWizard;
class MagWizard;
class ThermalWizard;
class WizardWidget;

class CalibrationPage : public QWidget {
    Q_OBJECT
public:
    explicit CalibrationPage(BoardModel *board, QWidget *parent = nullptr);
    CalFieldsWidget *fields() const { return m_fields; }
    MagWizard *magWizard() const { return m_mag; }
    ImuWizard *imuWizard() const { return m_imu; }
    ThermalWizard *thermalWizard() const { return m_thermal; }
    void showTab(int index);

private:
    QTabWidget *m_tabs;
    CalFieldsWidget *m_fields;
    MagWizard *m_mag;
    ImuWizard *m_imu;
    ThermalWizard *m_thermal;
    QList<WizardWidget *> m_wizards;
};

} // namespace ins10
