// Geometry of the 3D overview, kept out of the widget so it can be tested.
//
// The scene's world frame is ENU itself (x east, y north, z up). The board's
// orientation in it is the $ATT quaternion, normalised; the board is drawn in the
// body frame, which is the ICM-42688-P's: lying components up with J1 nearest to
// you, x points right (magnetometer -> barometer), y away from you (J1 -> INT pads),
// z up out of the component side.
#pragma once

#include "core/attitude.h"
#include "core/protocol.h"

#include <Eigen/Core>

#include <vector>

namespace ins10 {

// Board dimensions, mm (INS10-A3).
inline constexpr double kBoardLengthMm = 22.5;
inline constexpr double kBoardWidthMm = 13.2;
inline constexpr double kBoardThicknessMm = 1.6;
inline constexpr double kBoardCornerRadiusMm = 1.0;

// One part of the INS10-A3 as the 3D view draws it, in the body frame: mm from the
// centre of the outline. Positions from hardware/INS10-A.kicad_pcb, sizes from the
// parts' datasheets.
struct BoardPart {
    enum class Kind { Chip, Crystal, Capacitor, Pad };
    Kind kind;
    const char *reference;
    double x, y;          // centre
    double sizeX, sizeY;  // body; pads: copper diameter and hole diameter
    double height;        // above the component side; pads: 0
    int pin1Corner = -1;  // chips: corner of pin 1, 0 (-x,-y), 1 (+x,-y), 2 (+x,+y), 3 (-x,+y)
};
// U1-U3, Y1, C1-C8, J1.1-J1.8 (reference "J1", in pin order), J2-J4.
const std::vector<BoardPart> &boardParts();

struct SceneAttitude {
    Eigen::Vector4d q{1.0, 0.0, 0.0, 0.0}; // body -> ENU, unit length
    EulerAngles euler;
    double headingDeg = 90.0;
    bool valid = false;
};

// The attitude the scene draws for an $ATT record, and the angles that follow from it.
SceneAttitude sceneAttitude(const AttRecord &record);

// 4x4 model matrix (column vectors) placing the body frame in the world.
Eigen::Matrix4d bodyToWorld(const Eigen::Vector4d &q);

// Difference of two angles in degrees, wrapped to (-180, 180].
double angleDifference(double a, double b);

} // namespace ins10
