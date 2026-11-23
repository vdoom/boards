#include "app/language.h"

#include <QApplication>
#include <QLibraryInfo>
#include <QLocale>
#include <QSettings>
#include <QStringList>
#include <QTranslator>
#include <QWidget>

#include <memory>

namespace ins10::language {

namespace {
const QString kKey = QStringLiteral("ui/language");

struct State {
    QString code = QStringLiteral("en");
    bool applied = false;
    std::unique_ptr<QTranslator> app, qt;
};

State &state() {
    static State s;
    return s;
}

void removeTranslators() {
    State &s = state();
    for (std::unique_ptr<QTranslator> *t : {&s.app, &s.qt}) {
        if (*t) QCoreApplication::removeTranslator(t->get());
        t->reset();
    }
}

std::unique_ptr<QTranslator> loaded(const QString &name, const QString &directory) {
    auto translator = std::make_unique<QTranslator>();
    if (!translator->load(name, directory)) return nullptr;
    QCoreApplication::installTranslator(translator.get());
    return translator;
}
} // namespace

QString saved() {
    const QString choice = QSettings().value(kKey).toString();
    return choice == QLatin1String("en") || choice == QLatin1String("uk") ? choice : QString();
}

void save(const QString &choice) {
    QSettings settings;
    if (choice.isEmpty()) settings.remove(kKey);
    else settings.setValue(kKey, choice);
}

QString resolve(const QString &choice) {
    if (choice == QLatin1String("en") || choice == QLatin1String("uk")) return choice;
    const QStringList ui = QLocale::system().uiLanguages();
    return !ui.isEmpty() && ui.front().startsWith(QLatin1String("uk")) ? QStringLiteral("uk") : QStringLiteral("en");
}

QString active() { return state().code; }

void apply(const QString &code) {
    State &s = state();
    const QString wanted = code == QLatin1String("uk") ? code : QStringLiteral("en");
    if (s.applied && wanted == s.code) return;
    s.applied = true;
    removeTranslators();
    if (wanted == QLatin1String("uk")) {
        s.code = code;
        s.app = loaded(QStringLiteral("ins10studio_uk"), QStringLiteral(":/i18n"));
        s.qt = loaded(QStringLiteral("qtbase_uk"), QLibraryInfo::path(QLibraryInfo::TranslationsPath));
        QLocale::setDefault(QLocale(QLocale::Ukrainian, QLocale::Ukraine));
    } else {
        s.code = QStringLiteral("en");
        QLocale::setDefault(QLocale(QLocale::English, QLocale::UnitedStates));
    }
    emit notifier()->changed();
    // Texts drawn by paintEvent() follow at the next repaint.
    if (qobject_cast<QApplication *>(QCoreApplication::instance())) {
        for (QWidget *widget : QApplication::allWidgets()) widget->update();
    }
}

QString nativeName(const QString &code) {
    return code == QLatin1String("uk") ? QStringLiteral("Українська") : QStringLiteral("English");
}

Notifier *notifier() {
    static Notifier n;
    return &n;
}

} // namespace ins10::language
