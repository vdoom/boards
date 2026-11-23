// The interface language: the choice kept in the settings, what it means on this PC,
// and switching the running application between English (the language of the
// sources) and Ukrainian. Widgets set their texts through ui::translated() so that
// they follow a switch at once.
#pragma once

#include <QObject>
#include <QString>

namespace ins10::language {

// The choice kept in the settings: "" (the system's), "en" or "uk".
QString saved();
void save(const QString &choice);

// "uk" or "en". "" follows the system: Ukrainian when the system's interface is,
// else English.
QString resolve(const QString &choice);

// The language shown now: English until apply() says otherwise.
QString active();

// Shows the application in this language ("en" or "uk") from now on: installs the
// translators (this application's and Qt's own), sets the default locale for numbers,
// then tells every window to set its texts again.
void apply(const QString &code);

// "English", "Українська": what the language calls itself.
QString nativeName(const QString &code);

// Announces a switch, after the translators are in place.
class Notifier : public QObject {
    Q_OBJECT
signals:
    void changed();
};
Notifier *notifier();

} // namespace ins10::language
