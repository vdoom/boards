// One line of the ins10_ahrs USB console protocol, parsed into a typed record.
//
// Sources: docs/PROTOCOL.md and the printf formats in app/console.c and app/main.c of
// the firmware. Every format the application depends on is parsed here (and in
// status.h for the multi-line reports), so a change in the firmware's output needs
// changes in these two files only.
//
// Parsing is strict: wrong field counts, malformed numbers, non-finite or physically
// absurd values make a record "Other" with `damaged` set. The firmware drops output
// when the PC does not keep up, so truncated and concatenated lines do occur.
#pragma once

#include <QList>
#include <QString>
#include <QStringView>

#include <array>
#include <utility>
#include <variant>

namespace ins10 {

// $ATT,t_ms,qw,qx,qy,qz,roll,pitch,yaw,heading,bx,by,bz,tilt_sigma,heading_sigma,temp,flags
struct AttRecord {
    qint64 tMs = 0;
    std::array<double, 4> q{1.0, 0.0, 0.0, 0.0}; // w, x, y, z: body -> ENU
    double rollDeg = 0.0, pitchDeg = 0.0, yawDeg = 0.0, headingDeg = 0.0;
    std::array<double, 3> biasDps{};
    double tiltSigmaDeg = 0.0, headingSigmaDeg = 0.0;
    double tempC = 0.0;
    quint32 flags = 0;
};

// $RAW,seq,n,gx,gy,gz,ax,ay,az,temp: sums of n consecutive samples, sensor counts.
struct RawRecord {
    qint64 seq = 0;
    qint32 n = 1;
    std::array<qint64, 3> gyro{};
    std::array<qint64, 3> accel{};
    qint64 temp = 0;
};

// $MAG,t_ms,mx,my,mz: uT, chip axes, bridge offset removed, uncalibrated.
struct MagRecord {
    qint64 tMs = 0;
    std::array<double, 3> ut{};
};

// $BARO,t_ms,pressure_hPa,temp_C,altitude_m
struct BaroRecord {
    qint64 tMs = 0;
    double pressureHpa = 0.0, tempC = 0.0, altitudeM = 0.0;
};

// "# key=value key=value ..." (pairs may be empty for free text after '#').
struct InfoLine {
    QList<std::pair<QString, QString>> pairs;
};

// Printed by the firmware when a terminal opens the port:
// "# INS10 AHRS 2.0.0 (pico), library 2.0.0. Type 'help'."
struct BannerLine {
    QString name, version, board, library;
};

// "cal set <name> <values...>"
struct CalLine {
    QString name;
    QList<double> values;
};

struct OkLine {};
struct ErrLine {
    QString reason;
};

enum class LineKind { Att, Raw, Mag, Baro, Info, Banner, Cal, Ok, Err, Other };

struct ParsedLine {
    LineKind kind = LineKind::Other;
    QString text;          // the line as received, without terminator
    bool damaged = false;  // looked like protocol output but failed the checks
    std::variant<std::monostate, AttRecord, RawRecord, MagRecord, BaroRecord, InfoLine, BannerLine,
                 CalLine, OkLine, ErrLine>
        data;

    bool isStream() const {
        return kind == LineKind::Att || kind == LineKind::Raw || kind == LineKind::Mag ||
               kind == LineKind::Baro;
    }
    template <typename T> const T &as() const { return std::get<T>(data); }
};

ParsedLine parseLine(QStringView text);

// Python-compatible "# a=1 b=two" splitting (records.parse_info_line): tokens are
// separated by white space, a token with '=' and a non-empty key is a pair, and a
// trailing ',' is stripped from the value.
QList<std::pair<QString, QString>> parseInfoPairs(QStringView text);

// Plausibility limits applied by the parser.
namespace limits {
inline constexpr int kRawMaxN = 1024;
inline constexpr qint64 kRawMaxCount = 1 << 19;  // 20-bit FIFO samples
inline constexpr qint64 kTempMaxCount = 1 << 15; // 16-bit temperature
inline constexpr double kMagMaxUt = 1000.0;
inline constexpr double kTempMinC = -60.0, kTempMaxC = 150.0;
inline constexpr double kPressureMinHpa = 200.0, kPressureMaxHpa = 1300.0;
inline constexpr double kAltitudeMinM = -5000.0, kAltitudeMaxM = 20000.0;
inline constexpr double kBiasMaxDps = 500.0;
inline constexpr double kSigmaMaxDeg = 1000.0;
} // namespace limits

// Longest command line the firmware accepts, and its word limit (console.c).
inline constexpr int kMaxCommandLength = 199;
inline constexpr int kMaxCommandWords = 14;

} // namespace ins10
