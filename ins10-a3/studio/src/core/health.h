// Board health over time: a history of 'status' reports, and the automated first-run
// checklist from the firmware's README ("Перший запуск на платі", items 2-5 and 7).
#pragma once

#include "core/status.h"

#include <QList>
#include <QString>

#include <deque>
#include <functional>
#include <optional>

namespace ins10 {

struct StatusSample {
    double t = 0.0; // s, PC clock
    StatusReport report;
};

class StatusHistory {
public:
    explicit StatusHistory(size_t capacity = 3600) : m_capacity(capacity) {}
    void add(double t, const StatusReport &report);
    void clear() { m_samples.clear(); }
    bool isEmpty() const { return m_samples.empty(); }
    const std::deque<StatusSample> &samples() const { return m_samples; }
    const StatusSample *latest() const { return m_samples.empty() ? nullptr : &m_samples.back(); }

    using Pick = std::function<std::optional<double>(const StatusReport &)>;
    // Growth of a counter over (about) the last `window` seconds, per second; nothing
    // when the history is shorter than half the window or the counter went backwards
    // (the board restarted).
    std::optional<double> rate(const Pick &pick, double window) const;
    // Growth of a counter over the last `window` seconds.
    std::optional<double> growth(const Pick &pick, double window) const;
    // Span of time covered by the samples.
    double span() const { return m_samples.empty() ? 0.0 : m_samples.back().t - m_samples.front().t; }

private:
    std::optional<std::pair<const StatusSample *, const StatusSample *>> window(const Pick &pick, double seconds) const;
    size_t m_capacity;
    std::deque<StatusSample> m_samples;
};

enum class CheckLevel { Pending, Ok, Warning, Fail };

struct CheckResult {
    QString id;
    QString title;
    CheckLevel level = CheckLevel::Pending;
    QString detail;  // what was measured
    QString advice;  // what to look at when it is not right
};

// Evaluates every check from the latest 'info' and the status history. Checks that
// need a rate over time stay Pending until the history is long enough.
QList<CheckResult> evaluateFirstRun(const InfoReport &info, const StatusHistory &history);

} // namespace ins10
