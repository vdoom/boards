// The overview's 3D scene: ground grid with north and east marks, the INS10-A3 (22.5 x
// 13.2 x 1.6 mm, its parts where they are on the real board) and the IMU's X/Y/Z axes
// in red, green and blue.
#pragma once

#include "app/glscene.h"

#include <Eigen/Core>

namespace ins10 {

class BoardView : public GlScene {
    Q_OBJECT
public:
    explicit BoardView(QWidget *parent = nullptr);

    void setAttitude(const Eigen::Vector4d &q);
    Eigen::Vector4d attitude() const { return m_q; }

private:
    void updateModels();

    int m_board = -1, m_axes = -1;
    Eigen::Vector4d m_q{1.0, 0.0, 0.0, 0.0};
};

QMatrix4x4 toQMatrix(const Eigen::Matrix4d &m);

} // namespace ins10
