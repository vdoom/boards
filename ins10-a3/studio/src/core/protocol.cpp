#include "core/protocol.h"

#include <QRegularExpression>

#include <cmath>

namespace ins10 {

namespace {

// Splits on `sep` into at most `max` views. Returns the number of fields, or max + 1
// when there are more.
int splitFields(QStringView text, QChar sep, QStringView *out, int max) {
    int count = 0;
    qsizetype start = 0;
    for (qsizetype i = 0; i <= text.size(); ++i) {
        if (i == text.size() || text[i] == sep) {
            if (count == max) return max + 1;
            out[count++] = text.mid(start, i - start);
            start = i + 1;
        }
    }
    return count;
}

bool hasSpace(QStringView s) {
    for (QChar c : s) {
        if (c.isSpace()) return true;
    }
    return false;
}

bool toDouble(QStringView s, double &value) {
    if (s.isEmpty() || hasSpace(s)) return false;
    bool ok = false;
    value = s.toDouble(&ok);
    return ok && std::isfinite(value);
}

bool toInt64(QStringView s, qint64 &value) {
    if (s.isEmpty() || hasSpace(s)) return false;
    bool ok = false;
    value = s.toLongLong(&ok, 10);
    return ok;
}

bool toHex(QStringView s, quint32 &value) {
    if (s.isEmpty() || s.size() > 8) return false;
    for (QChar c : s) {
        const char16_t u = c.unicode();
        const bool hex = (u >= '0' && u <= '9') || (u >= 'a' && u <= 'f') || (u >= 'A' && u <= 'F');
        if (!hex) return false;
    }
    bool ok = false;
    value = s.toUInt(&ok, 16);
    return ok;
}

bool within(double v, double lo, double hi) { return v >= lo && v <= hi; }

const QRegularExpression &whiteSpace() {
    static const QRegularExpression re(QStringLiteral("\\s+"));
    return re;
}

bool parseAtt(QStringView text, AttRecord &r) {
    QStringView f[17];
    if (splitFields(text, u',', f, 17) != 17) return false;
    double v[15];
    if (!toInt64(f[1], r.tMs) || r.tMs < 0) return false;
    for (int i = 0; i < 14; ++i) {
        if (!toDouble(f[2 + i], v[i])) return false;
    }
    // The firmware prints flags with %03X; a shorter field is a truncated line.
    if (f[16].size() < 3 || !toHex(f[16], r.flags)) return false;
    for (int i = 0; i < 4; ++i) r.q[static_cast<size_t>(i)] = v[i];
    const double norm = std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2] + v[3] * v[3]);
    if (!within(norm, 0.9, 1.1)) return false;
    r.rollDeg = v[4];
    r.pitchDeg = v[5];
    r.yawDeg = v[6];
    r.headingDeg = v[7];
    if (!within(r.rollDeg, -180.01, 180.01) || !within(r.pitchDeg, -90.01, 90.01) ||
        !within(r.yawDeg, -180.01, 180.01) || !within(r.headingDeg, -0.01, 360.01))
        return false;
    for (int i = 0; i < 3; ++i) {
        r.biasDps[static_cast<size_t>(i)] = v[8 + i];
        if (std::abs(v[8 + i]) > limits::kBiasMaxDps) return false;
    }
    r.tiltSigmaDeg = v[11];
    r.headingSigmaDeg = v[12];
    r.tempC = v[13];
    if (!within(r.tiltSigmaDeg, 0.0, limits::kSigmaMaxDeg) ||
        !within(r.headingSigmaDeg, 0.0, limits::kSigmaMaxDeg) ||
        !within(r.tempC, limits::kTempMinC, limits::kTempMaxC))
        return false;
    return true;
}

bool parseRaw(QStringView text, RawRecord &r) {
    QStringView f[10];
    if (splitFields(text, u',', f, 10) != 10) return false;
    qint64 v[9];
    for (int i = 0; i < 9; ++i) {
        if (!toInt64(f[1 + i], v[i])) return false;
    }
    if (v[0] < 0 || v[1] < 1 || v[1] > limits::kRawMaxN) return false;
    r.seq = v[0];
    r.n = static_cast<qint32>(v[1]);
    const qint64 maxSum = limits::kRawMaxCount * r.n;
    for (int i = 0; i < 3; ++i) {
        r.gyro[static_cast<size_t>(i)] = v[2 + i];
        r.accel[static_cast<size_t>(i)] = v[5 + i];
        if (std::abs(v[2 + i]) > maxSum || std::abs(v[5 + i]) > maxSum) return false;
    }
    r.temp = v[8];
    if (std::abs(r.temp) > limits::kTempMaxCount * r.n) return false;
    return true;
}

bool parseMag(QStringView text, MagRecord &r) {
    QStringView f[5];
    if (splitFields(text, u',', f, 5) != 5) return false;
    if (!toInt64(f[1], r.tMs) || r.tMs < 0) return false;
    for (int i = 0; i < 3; ++i) {
        double v;
        if (!toDouble(f[2 + i], v) || std::abs(v) > limits::kMagMaxUt) return false;
        r.ut[static_cast<size_t>(i)] = v;
    }
    return true;
}

bool parseBaro(QStringView text, BaroRecord &r) {
    QStringView f[5];
    if (splitFields(text, u',', f, 5) != 5) return false;
    if (!toInt64(f[1], r.tMs) || r.tMs < 0) return false;
    if (!toDouble(f[2], r.pressureHpa) || !toDouble(f[3], r.tempC) || !toDouble(f[4], r.altitudeM))
        return false;
    return within(r.pressureHpa, limits::kPressureMinHpa, limits::kPressureMaxHpa) &&
           within(r.tempC, limits::kTempMinC, limits::kTempMaxC) &&
           within(r.altitudeM, limits::kAltitudeMinM, limits::kAltitudeMaxM);
}

bool parseCal(QStringView text, CalLine &r) {
    const QList<QStringView> parts = text.split(whiteSpace(), Qt::SkipEmptyParts);
    if (parts.size() < 4 || parts[0] != u"cal" || parts[1] != u"set") return false;
    r.name = parts[2].toString();
    r.values.clear();
    for (qsizetype i = 3; i < parts.size(); ++i) {
        double v;
        if (!toDouble(parts[i], v)) return false;
        r.values.append(v);
    }
    return true;
}

bool parseBanner(QStringView text, BannerLine &r) {
    static const QRegularExpression re(
        QStringLiteral("^# (.+) (\\S+) \\((\\S+)\\), library (\\S+)\\. Type 'help'\\.$"));
    const QRegularExpressionMatch m = re.matchView(text);
    if (!m.hasMatch()) return false;
    r.name = m.captured(1);
    r.version = m.captured(2);
    r.board = m.captured(3);
    r.library = m.captured(4);
    return true;
}

bool printableAscii(QStringView text) {
    for (QChar c : text) {
        const char16_t u = c.unicode();
        if (u < 0x20 || u > 0x7e) {
            if (u != '\t') return false;
        }
    }
    return true;
}

} // namespace

QList<std::pair<QString, QString>> parseInfoPairs(QStringView text) {
    QList<std::pair<QString, QString>> out;
    if (!text.startsWith(u'#')) return out;
    const QList<QStringView> tokens = text.mid(1).split(whiteSpace(), Qt::SkipEmptyParts);
    for (QStringView token : tokens) {
        const qsizetype eq = token.indexOf(u'=');
        if (eq <= 0) continue;
        QStringView value = token.mid(eq + 1);
        while (value.endsWith(u',')) value.chop(1);
        out.append({token.left(eq).toString(), value.toString()});
    }
    return out;
}

ParsedLine parseLine(QStringView text) {
    ParsedLine line;
    while (text.endsWith(u'\r') || text.endsWith(u'\n')) text.chop(1);
    line.text = text.toString();
    if (text.isEmpty()) return line;
    if (!printableAscii(text) || text.indexOf(u'$', 1) >= 0) {
        line.damaged = true;
        return line;
    }
    if (text.startsWith(u'$')) {
        if (text.startsWith(u"$ATT,")) {
            AttRecord r;
            if (parseAtt(text, r)) {
                line.kind = LineKind::Att;
                line.data = r;
                return line;
            }
        } else if (text.startsWith(u"$RAW,")) {
            RawRecord r;
            if (parseRaw(text, r)) {
                line.kind = LineKind::Raw;
                line.data = r;
                return line;
            }
        } else if (text.startsWith(u"$MAG,")) {
            MagRecord r;
            if (parseMag(text, r)) {
                line.kind = LineKind::Mag;
                line.data = r;
                return line;
            }
        } else if (text.startsWith(u"$BARO,")) {
            BaroRecord r;
            if (parseBaro(text, r)) {
                line.kind = LineKind::Baro;
                line.data = r;
                return line;
            }
        }
        line.damaged = true;
        return line;
    }
    if (text.startsWith(u'#')) {
        BannerLine banner;
        if (parseBanner(text, banner)) {
            line.kind = LineKind::Banner;
            line.data = banner;
            return line;
        }
        line.kind = LineKind::Info;
        line.data = InfoLine{parseInfoPairs(text)};
        return line;
    }
    if (text == u"OK" || text.startsWith(u"OK ")) {
        line.kind = LineKind::Ok;
        line.data = OkLine{};
        return line;
    }
    if (text == u"ERR" || text.startsWith(u"ERR ")) {
        line.kind = LineKind::Err;
        line.data = ErrLine{text.mid(3).trimmed().toString()};
        return line;
    }
    if (text.startsWith(u"cal set ")) {
        CalLine r;
        if (parseCal(text, r)) {
            line.kind = LineKind::Cal;
            line.data = r;
            return line;
        }
    }
    // Nothing the firmware prints looks like this: the remains of a damaged line, or
    // another firmware on the port.
    line.damaged = true;
    return line;
}

} // namespace ins10
