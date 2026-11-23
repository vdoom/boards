// Recordings on disk: plain protocol text, one line per record, in the layout of
// `ins10.py log` (the '#' lines of 'info' first, then the stream). Such files are read
// by `ins10.py ... --from-log` and by the firmware's `ins10_replay`.
#pragma once

#include "core/recording.h"

#include <QFile>
#include <QString>
#include <QStringList>

#include <memory>
#include <optional>

namespace ins10 {

// Writes lines as they arrive. Lines are ASCII; the terminator is '\n'.
class LogWriter {
public:
    LogWriter();
    ~LogWriter();
    LogWriter(const LogWriter &) = delete;
    LogWriter &operator=(const LogWriter &) = delete;

    bool open(const QString &path, QString *error = nullptr);
    bool isOpen() const;
    QString path() const;
    void writeLine(QStringView line);
    void writeLines(const QStringList &lines);
    // Flushes and closes; false (with a reason) if anything failed to reach the disk.
    bool close(QString *error = nullptr);

    qint64 linesWritten() const { return m_lines; }
    qint64 bytesWritten() const { return m_bytes; }

private:
    std::unique_ptr<QFile> m_file;
    qint64 m_lines = 0;
    qint64 m_bytes = 0;
    bool m_failed = false;
};

// Whole file at once, replaced atomically.
bool writeLogFile(const QString &path, const QStringList &lines, QString *error = nullptr);
std::optional<QStringList> readLogFile(const QString &path, QString *error = nullptr);
// records.load_log
std::optional<Recording> loadLog(const QString &path, QString *error = nullptr);

} // namespace ins10
