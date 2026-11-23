#include "app/settingsdialog.h"

#include "app/language.h"

#include <QComboBox>
#include <QDialogButtonBox>
#include <QFormLayout>
#include <QLabel>
#include <QSettings>
#include <QSpinBox>
#include <QVBoxLayout>

namespace ins10 {

int SettingsDialog::viewAttRate() { return std::clamp(QSettings().value(QStringLiteral("view/attRateHz"), 50).toInt(), 1, 200); }

SettingsDialog::SettingsDialog(QWidget *parent)
    : QDialog(parent), m_language(new QComboBox(this)), m_rate(new QSpinBox(this)) {
    setWindowTitle(tr("Settings"));
    auto *layout = new QVBoxLayout(this);
    auto *form = new QFormLayout;
    m_language->addItem(tr("As the system (%1)").arg(language::nativeName(language::resolve(QString()))), QString());
    for (const char *code : {"en", "uk"}) {
        m_language->addItem(language::nativeName(QLatin1String(code)), QString::fromLatin1(code));
    }
    m_language->setCurrentIndex(std::max(0, m_language->findData(language::saved())));
    form->addRow(tr("Language"), m_language);
    m_rate->setRange(1, 200);
    m_rate->setSuffix(tr(" Hz"));
    m_rate->setValue(viewAttRate());
    m_rate->setToolTip(tr("While connected the application sets this $ATT rate for its views. The board's own rate is "
                          "put back before every 'cal save' and when disconnecting."));
    form->addRow(tr("Attitude rate for viewing"), m_rate);
    layout->addLayout(form);
    auto *buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, this);
    connect(buttons, &QDialogButtonBox::accepted, this, &QDialog::accept);
    connect(buttons, &QDialogButtonBox::rejected, this, &QDialog::reject);
    layout->addWidget(buttons);
}

// The language is only saved here; MainWindow switches to it.
void SettingsDialog::accept() {
    language::save(m_language->currentData().toString());
    QSettings settings;
    const bool rateChanged = m_rate->value() != viewAttRate();
    settings.setValue(QStringLiteral("view/attRateHz"), m_rate->value());
    if (rateChanged) emit viewAttRateChanged(m_rate->value());
    QDialog::accept();
}

} // namespace ins10
