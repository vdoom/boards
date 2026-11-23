// 3D views of the calibration wizards: a unit sphere (wire grid, optionally its 72
// equal-area cells coloured visited/empty), point clouds and markers on it.
#pragma once

#include "app/glscene.h"

#include <Eigen/Core>

#include <vector>

namespace ins10 {

class SphereView : public GlScene {
    Q_OBJECT
public:
    explicit SphereView(QWidget *parent = nullptr);

    // Cells of sphereCoverage (6 bands of equal z x 12 sectors); empty to hide.
    void setCells(const std::vector<bool> &cells);
    // Up to two point clouds (0 and 1), already scaled to about unit radius.
    void setCloud(int index, const Eigen::MatrixX3d &points, const QColor &color, float size = 3.0f);
    void clearCloud(int index);
    // Big dots, e.g. gravity directions of rest positions.
    void setMarkers(const std::vector<Eigen::Vector3d> &points, const QColor &color);
    // Labels at the six axis directions.
    void showAxisLabels(bool show);

private:
    int m_cells = -1, m_clouds[2] = {-1, -1}, m_markers = -1;
};

} // namespace ins10
