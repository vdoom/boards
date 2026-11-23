#include "app/calibrationpage.h"

#include "app/calfieldswidget.h"
#include "app/calwizards.h"
#include "app/uiutil.h"

#include <QLabel>
#include <QTabWidget>
#include <QVBoxLayout>

namespace ins10 {

CalibrationPage::CalibrationPage(BoardModel *board, QWidget *parent)
    : QWidget(parent), m_tabs(new QTabWidget(this)), m_fields(new CalFieldsWidget(board, this)),
      m_mag(new MagWizard(board, this)), m_imu(new ImuWizard(board, this)), m_thermal(new ThermalWizard(board, this)) {
    auto *layout = new QVBoxLayout(this);
    layout->setContentsMargins(0, 0, 0, 0);
    auto *order = new QLabel(this);
    order->setWordWrap(true);
    order->setContentsMargins(8, 6, 8, 0);
    layout->addWidget(order);
    m_tabs->addTab(m_fields, QString());
    m_tabs->addTab(m_imu, QString());
    m_tabs->addTab(m_mag, QString());
    m_tabs->addTab(m_thermal, QString());
    layout->addWidget(m_tabs);
    ui::translated(this, [this, order] {
        order->setText(tr("Recommended order: accelerometer and gyro first, then the magnetometer (it is aligned to the "
                          "accelerometer's axes), then, if wanted, temperature."));
        m_tabs->setTabText(0, tr("Fields"));
        m_tabs->setTabText(1, tr("Accelerometer and gyro"));
        m_tabs->setTabText(2, tr("Magnetometer"));
        m_tabs->setTabText(3, tr("Temperature"));
    });
    m_wizards = {m_imu, m_mag, m_thermal};
    for (WizardWidget *w : m_wizards) {
        connect(w, &WizardWidget::busyChanged, this, [this, w](bool busy) {
            for (WizardWidget *other : m_wizards) {
                if (other != w) other->setBlocked(busy);
            }
        });
    }
}

void CalibrationPage::showTab(int index) { m_tabs->setCurrentIndex(index); }

} // namespace ins10
