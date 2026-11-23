// Errors and warnings of the calibration algorithms. Texts are translated; the codes
// let callers and tests tell them apart without parsing.
#pragma once

#include <QString>
#include <QStringList>

#include <stdexcept>

namespace ins10::calib {

class CalibrationError : public std::runtime_error {
public:
    enum class Code {
        TooFewSamples,
        NoSpread,
        NotAnEllipsoid,
        DegenerateFit,
        SoftIronNotPositive,
        TooFewOrientations,
        AccelOutOfTolerance,
        TooFewRestPositions,
        RecordingTooShort,
        TemperatureSpanTooSmall,
        Cancelled,
    };
    CalibrationError(Code code, const QString &message)
        : std::runtime_error(message.toStdString()), m_code(code), m_message(message) {}
    Code code() const { return m_code; }
    const QString &message() const { return m_message; }

private:
    Code m_code;
    QString m_message;
};

struct Warning {
    enum class Code {
        CoverageLow,
        FieldOutOfRange,
        ResidualHigh,
        TooFewSmoothSamples,
        AlignmentPoor,
        DipVaries,
        AccelUncalibrated,
        DipNegative,
        Mirrored,
        FewOrientations,
        GyroNotFitted,
        GyroScaleOnly,
        AccelResidualHigh,
        ThermalOrderLow,
        ThermalMoved,
    };
    Code code;
    QString text;
};
using Warnings = QList<Warning>;

QStringList warningTexts(const Warnings &warnings);
bool hasWarning(const Warnings &warnings, Warning::Code code);

} // namespace ins10::calib
