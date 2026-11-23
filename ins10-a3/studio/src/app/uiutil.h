// Small helpers shared by the pages: colours, icons, fonts, number formatting.
#pragma once

#include "core/health.h"

#include <QColor>
#include <QFont>
#include <QIcon>
#include <QString>

#include <functional>

class QObject;
class QWidget;

namespace ins10::ui {

// Runs apply() now, and again each time the interface language changes, for as long
// as owner exists. Everything that sets translated text once (labels, buttons, tool
// tips, menus, headers) goes through here, so that a language switch takes effect
// without a restart.
void translated(QObject *owner, std::function<void()> apply);

QColor levelColor(CheckLevel level);
QIcon levelIcon(CheckLevel level);
QString levelText(CheckLevel level);
QIcon dotIcon(const QColor &color, int size = 12);
QFont monospaceFont();
// Locale-aware fixed-point number; "-" for NaN.
QString number(double value, int decimals);
// Whether the palette is dark (text lighter than the window).
bool darkPalette(const QWidget *widget);

// Status flag bit -> colour when set: green for good news, amber for "watch this",
// red for trouble.
QColor flagColor(int bit);
// Short description of status flag bit `bit` for tool tips.
QString flagDescription(int bit);

} // namespace ins10::ui
