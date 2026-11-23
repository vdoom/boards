#include "app/uiutil.h"

#include "app/language.h"

#include <QApplication>
#include <QFontDatabase>
#include <QLocale>
#include <QPainter>
#include <QPixmap>
#include <QWidget>

#include <cmath>

namespace ins10::ui {

void translated(QObject *owner, std::function<void()> apply) {
    apply();
    QObject::connect(language::notifier(), &language::Notifier::changed, owner, std::move(apply));
}

QColor levelColor(CheckLevel level) {
    switch (level) {
    case CheckLevel::Ok: return QColor(0x2e, 0x9d, 0x4f);
    case CheckLevel::Warning: return QColor(0xe0, 0x9b, 0x12);
    case CheckLevel::Fail: return QColor(0xd2, 0x3a, 0x33);
    case CheckLevel::Pending: break;
    }
    return QColor(0x8a, 0x8f, 0x98);
}

QIcon dotIcon(const QColor &color, int size) {
    QPixmap pixmap(size * 2, size * 2);
    pixmap.setDevicePixelRatio(2.0);
    pixmap.fill(Qt::transparent);
    QPainter p(&pixmap);
    p.setRenderHint(QPainter::Antialiasing);
    p.setPen(color.darker(130));
    p.setBrush(color);
    p.drawEllipse(QRectF(1.0, 1.0, size - 2.0, size - 2.0));
    return QIcon(pixmap);
}

QIcon levelIcon(CheckLevel level) { return dotIcon(levelColor(level), 14); }

QString levelText(CheckLevel level) {
    switch (level) {
    case CheckLevel::Ok: return QApplication::translate("ins10::ui", "OK");
    case CheckLevel::Warning: return QApplication::translate("ins10::ui", "Attention");
    case CheckLevel::Fail: return QApplication::translate("ins10::ui", "Failed");
    case CheckLevel::Pending: break;
    }
    return QApplication::translate("ins10::ui", "Waiting");
}

QFont monospaceFont() { return QFontDatabase::systemFont(QFontDatabase::FixedFont); }

QString number(double value, int decimals) {
    if (!std::isfinite(value)) return QStringLiteral("-");
    return QLocale().toString(value, 'f', decimals);
}

bool darkPalette(const QWidget *widget) {
    const QPalette pal = widget ? widget->palette() : QApplication::palette();
    return pal.color(QPalette::WindowText).lightness() > pal.color(QPalette::Window).lightness();
}

QColor flagColor(int bit) {
    switch (bit) {
    case 0: case 2: case 3: return QColor(0x2e, 0x9d, 0x4f);           // aligned, still, heading
    case 1: case 4: case 5: case 8: return QColor(0xe0, 0x9b, 0x12);    // coarse, disturbances, thermal
    default: return QColor(0xd2, 0x3a, 0x33);                           // mag-uncal, saturated, recovery, mag-stale
    }
}

QString flagDescription(int bit) {
    static const char *const text[] = {
        QT_TRANSLATE_NOOP("ins10::ui", "Attitude initialised"),
        QT_TRANSLATE_NOOP("ins10::ui", "Aligned while moving; tilt still converging"),
        QT_TRANSLATE_NOOP("ins10::ui", "Board at rest; zero-rate updates active"),
        QT_TRANSLATE_NOOP("ins10::ui", "Heading referenced to the magnetic field"),
        QT_TRANSLATE_NOOP("ins10::ui", "Acceleration detected; accelerometer weight reduced at least tenfold"),
        QT_TRANSLATE_NOOP("ins10::ui", "Magnetic disturbance detected; magnetometer weight reduced"),
        QT_TRANSLATE_NOOP("ins10::ui", "Magnetometer ignored: not calibrated"),
        QT_TRANSLATE_NOOP("ins10::ui", "Sensor saturated or samples lost in the last second"),
        QT_TRANSLATE_NOOP("ins10::ui", "Temperature transient in progress"),
        QT_TRANSLATE_NOOP("ins10::ui", "Divergence watchdog acted in the last 5 s"),
        QT_TRANSLATE_NOOP("ins10::ui", "No usable magnetometer sample for over a second"),
    };
    if (bit < 0 || bit >= static_cast<int>(std::size(text))) return {};
    return QApplication::translate("ins10::ui", text[bit]);
}

} // namespace ins10::ui
