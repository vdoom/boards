// The three calibration wizards: magnetometer, accelerometer and gyro, temperature.
#pragma once

#include "app/wizardwidget.h"
#include "core/series.h"

#include <memory>

class QCheckBox;
class QLabel;
class QTreeWidget;

namespace ins10 {

class PlotWidget;
class SphereView;
class TimeView;
class XyPlot;

class MagWizard : public WizardWidget {
    Q_OBJECT
public:
    explicit MagWizard(BoardModel *board, QWidget *parent = nullptr);

protected:
    QString instructions() const override;
    QWidget *createLiveView() override;
    QWidget *createFigures() override;
    void updateLive() override;
    void showResult() override;
    QString resultHtml() const override;
    void reset() override;

private:
    SphereView *m_view = nullptr;
    QLabel *m_samples = nullptr, *m_coverage = nullptr, *m_speed = nullptr;
    QCheckBox *m_southern = nullptr;
};

class ImuWizard : public WizardWidget {
    Q_OBJECT
public:
    explicit ImuWizard(BoardModel *board, QWidget *parent = nullptr);

protected:
    QString instructions() const override;
    QWidget *createLiveView() override;
    QWidget *createFigures() override;
    void updateLive() override;
    void showResult() override;
    QString resultHtml() const override;
    void reset() override;

private:
    SphereView *m_view = nullptr;
    QTreeWidget *m_positions = nullptr;
    QLabel *m_count = nullptr, *m_orientations = nullptr, *m_advice = nullptr;
};

class ThermalWizard : public WizardWidget {
    Q_OBJECT
public:
    explicit ThermalWizard(BoardModel *board, QWidget *parent = nullptr);
    ~ThermalWizard() override;

protected:
    QString instructions() const override;
    QWidget *createLiveView() override;
    QWidget *createFigures() override;
    void updateLive() override;
    void showResult() override;
    QString resultHtml() const override;
    void reset() override;

private:
    void updateScatter(bool withFit);

    std::unique_ptr<TimeSeriesGroup> m_series;
    TimeView *m_timeView = nullptr;
    PlotWidget *m_tempPlot = nullptr;
    XyPlot *m_scatter = nullptr;
    QLabel *m_now = nullptr, *m_range = nullptr, *m_motion = nullptr;
    qsizetype m_rowsShown = 0;
};

} // namespace ins10
