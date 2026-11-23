#include "core/logfile.h"

#include <QCoreApplication>
#include <QSaveFile>

namespace ins10 {

namespace {
struct Text {
    Q_DECLARE_TR_FUNCTIONS(ins10::LogFile)
};
} // namespace

LogWriter::LogWriter() = default;
LogWriter::~LogWriter() { close(); }

bool LogWriter::open(const QString &path, QString *error) {
    close();
    m_lines = m_bytes = 0;
    m_failed = false;
    m_file = std::make_unique<QFile>(path);
    if (!m_file->open(QIODevice::WriteOnly | QIODevice::Truncate)) {
        if (error) *error = m_file->errorString();
        m_file.reset();
        return false;
    }
    return true;
}

bool LogWriter::isOpen() const { return m_file && m_file->isOpen(); }

QString LogWriter::path() const { return m_file ? m_file->fileName() : QString(); }

void LogWriter::writeLine(QStringView line) {
    if (!isOpen()) return;
    QByteArray bytes = line.toLatin1();
    bytes.append('\n');
    if (m_file->write(bytes) != bytes.size()) m_failed = true;
    ++m_lines;
    m_bytes += bytes.size();
}

void LogWriter::writeLines(const QStringList &lines) {
    for (const QString &line : lines) writeLine(line);
}

bool LogWriter::close(QString *error) {
    if (!m_file) return true;
    bool ok = !m_failed && m_file->flush();
    const QString reason = m_file->errorString();
    m_file->close();
    m_file.reset();
    if (!ok && error) *error = reason.isEmpty() ? Text::tr("write failed") : reason;
    return ok;
}

bool writeLogFile(const QString &path, const QStringList &lines, QString *error) {
    QSaveFile file(path);
    if (!file.open(QIODevice::WriteOnly)) {
        if (error) *error = file.errorString();
        return false;
    }
    for (const QString &line : lines) {
        QByteArray bytes = line.toLatin1();
        bytes.append('\n');
        file.write(bytes);
    }
    if (!file.commit()) {
        if (error) *error = file.errorString();
        return false;
    }
    return true;
}

std::optional<QStringList> readLogFile(const QString &path, QString *error) {
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly)) {
        if (error) *error = file.errorString();
        return std::nullopt;
    }
    QStringList lines;
    while (!file.atEnd()) {
        QByteArray line = file.readLine();
        while (line.endsWith('\n') || line.endsWith('\r')) line.chop(1);
        lines.append(QString::fromUtf8(line));
    }
    return lines;
}

std::optional<Recording> loadLog(const QString &path, QString *error) {
    const auto lines = readLogFile(path, error);
    if (!lines) return std::nullopt;
    Recording rec;
    for (const QString &line : *lines) rec.feed(line);
    return rec;
}

} // namespace ins10
