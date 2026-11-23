#include "app/boardview.h"

#include "app/uiutil.h"
#include "core/scene.h"

#include <QVector2D>

#include <cmath>
#include <numbers>

namespace ins10 {

namespace {

constexpr float kUnitsPerMm = 0.1f; // the scene's unit is 10 mm

// Rounded rectangle of half-sizes hx, hy and corner radius r, counter-clockwise seen
// from +z.
std::vector<QVector2D> roundedRectangle(float hx, float hy, float r) {
    constexpr int kSteps = 6;
    const QVector2D centres[4] = {{hx - r, -hy + r}, {hx - r, hy - r}, {-hx + r, hy - r}, {-hx + r, -hy + r}};
    std::vector<QVector2D> outline;
    for (int c = 0; c < 4; ++c) {
        for (int i = 0; i <= kSteps; ++i) {
            const float a = static_cast<float>(std::numbers::pi / 2 * ((c - 1) + static_cast<double>(i) / kSteps));
            outline.push_back(centres[c] + r * QVector2D(std::cos(a), std::sin(a)));
        }
    }
    return outline;
}

// The outline extruded from z0 to z1.
void slab(MeshData &mesh, const std::vector<QVector2D> &outline, float z0, float z1, const QColor &top,
          const QColor &bottom, const QColor &side) {
    const size_t n = outline.size();
    for (size_t i = 0; i < n; ++i) {
        const QVector2D a = outline[i], b = outline[(i + 1) % n];
        mesh.triangle({0, 0, z1}, {a.x(), a.y(), z1}, {b.x(), b.y(), z1}, top);
        mesh.triangle({0, 0, z0}, {b.x(), b.y(), z0}, {a.x(), a.y(), z0}, bottom);
        mesh.quad({a.x(), a.y(), z0}, {b.x(), b.y(), z0}, {b.x(), b.y(), z1}, {a.x(), a.y(), z1}, side);
    }
}

// Flat disc at height z facing up (or down).
void disc(MeshData &mesh, float cx, float cy, float radius, float z, bool up, const QColor &color) {
    constexpr int kSegments = 20;
    for (int i = 0; i < kSegments; ++i) {
        const double a0 = 2 * std::numbers::pi * i / kSegments, a1 = 2 * std::numbers::pi * (i + 1) / kSegments;
        const QVector3D c(cx, cy, z), p0(cx + radius * static_cast<float>(std::cos(a0)), cy + radius * static_cast<float>(std::sin(a0)), z),
            p1(cx + radius * static_cast<float>(std::cos(a1)), cy + radius * static_cast<float>(std::sin(a1)), z);
        if (up) mesh.triangle(c, p0, p1, color);
        else mesh.triangle(c, p1, p0, color);
    }
}

// The INS10-A3 from boardParts(), centred on the origin.
MeshData boardMesh() {
    const float lx = static_cast<float>(kBoardLengthMm) * kUnitsPerMm, ly = static_cast<float>(kBoardWidthMm) * kUnitsPerMm,
                lz = static_cast<float>(kBoardThicknessMm) * kUnitsPerMm;
    const float top = lz / 2, bottom = -lz / 2;
    const QColor mask(35, 112, 70), maskUnder(25, 80, 50), edge(200, 190, 140), gold(226, 184, 74), hole(28, 28, 30);
    const QColor chip(38, 38, 42), chipSide(58, 58, 64), lid(200, 200, 206), lidSide(160, 160, 168), ceramic(176, 146, 98),
        terminal(205, 205, 210), pin1(235, 235, 235);
    MeshData mesh;
    slab(mesh, roundedRectangle(lx / 2, ly / 2, static_cast<float>(kBoardCornerRadiusMm) * kUnitsPerMm), bottom, top,
         mask, maskUnder, edge);
    const float lift = 0.002f; // keeps coplanar faces apart in the depth buffer
    for (const BoardPart &part : boardParts()) {
        const float x = static_cast<float>(part.x) * kUnitsPerMm, y = static_cast<float>(part.y) * kUnitsPerMm;
        const float hx = static_cast<float>(part.sizeX) * kUnitsPerMm / 2, hy = static_cast<float>(part.sizeY) * kUnitsPerMm / 2;
        const float h = static_cast<float>(part.height) * kUnitsPerMm;
        switch (part.kind) {
        case BoardPart::Kind::Pad: // plated through hole: copper ring on both sides, sizeY is the hole
            for (const bool up : {true, false}) {
                const float z = up ? top : bottom, s = up ? 1.0f : -1.0f;
                disc(mesh, x, y, hx, z + s * lift, up, gold);
                disc(mesh, x, y, hy, z + s * 2 * lift, up, hole);
            }
            break;
        case BoardPart::Kind::Capacitor: {
            mesh.box({x - hx, y - hy, top}, {x + hx, y + hy, top + h}, ceramic, ceramic, ceramic);
            // Metal terminals at both ends of the longer side.
            const bool alongX = hx >= hy;
            const float t = (alongX ? hx : hy) * 0.45f;
            for (const float sgn : {-1.0f, 1.0f}) {
                const QVector3D c(alongX ? x + sgn * (hx - t / 2) : x, alongX ? y : y + sgn * (hy - t / 2), 0);
                const float ex = alongX ? t / 2 + lift : hx + lift, ey = alongX ? hy + lift : t / 2 + lift;
                mesh.box({c.x() - ex, c.y() - ey, top}, {c.x() + ex, c.y() + ey, top + h + lift}, terminal, terminal,
                         terminal);
            }
            break;
        }
        case BoardPart::Kind::Crystal:
            mesh.box({x - hx, y - hy, top}, {x + hx, y + hy, top + h}, lid, lid, lidSide);
            break;
        case BoardPart::Kind::Chip:
            mesh.box({x - hx, y - hy, top}, {x + hx, y + hy, top + h}, chip, chip, chipSide);
            break;
        }
        if (part.pin1Corner >= 0) {
            const float d = 0.045f, m = 0.012f; // inset and half-size of the pin-1 dot
            const float px = (part.pin1Corner == 1 || part.pin1Corner == 2) ? x + hx - d : x - hx + d;
            const float py = (part.pin1Corner >= 2) ? y + hy - d : y - hy + d;
            const float z = top + h + lift;
            mesh.quad({px - m, py - m, z}, {px + m, py - m, z}, {px + m, py + m, z}, {px - m, py + m, z}, pin1);
        }
    }
    return mesh;
}

} // namespace

QMatrix4x4 toQMatrix(const Eigen::Matrix4d &m) {
    QMatrix4x4 q;
    for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 4; ++c) q(r, c) = static_cast<float>(m(r, c));
    }
    return q;
}

BoardView::BoardView(QWidget *parent) : GlScene(parent) {
    // Far enough for the whole axis triad and its labels to stay inside the view in
    // every attitude (the Z label used to be cut off at the top).
    setDefaultView(-115.0f, 28.0f, 8.6f);

    // Ground: grid below the board, north (+y) and east (+x) marked.
    const float ground = -1.4f, extent = 4.0f;
    MeshData grid;
    grid.kind = MeshData::Kind::Lines;
    const QColor gridColor(150, 150, 150);
    for (float v = -extent; v <= extent + 1e-3f; v += 0.5f) {
        grid.line({v, -extent, ground}, {v, extent, ground}, gridColor);
        grid.line({-extent, v, ground}, {extent, v, ground}, gridColor);
    }
    addMesh(grid);
    MeshData compass;
    compass.kind = MeshData::Kind::Triangles;
    const QColor north(205, 50, 45), east(60, 110, 200), other(130, 130, 130);
    const float z = ground + 0.01f;
    compass.triangle({-0.18f, 0, z}, {0.18f, 0, z}, {0, extent, z}, north);
    compass.triangle({0, -0.18f, z}, {extent, 0, z}, {0, 0.18f, z}, east);
    compass.triangle({0.12f, 0, z}, {-0.12f, 0, z}, {0, -extent * 0.7f, z}, other);
    compass.triangle({0, 0.12f, z}, {-extent * 0.7f, 0, z}, {0, -0.12f, z}, other);
    addMesh(compass);

    // The board as it is: J1 along the near long edge, the INT pads along the far one,
    // magnetometer, IMU and barometer from left to right (see boardParts()).
    m_board = addMesh(boardMesh(), true);

    // Chip axes.
    MeshData axes;
    // Unlit: the arrows keep their pure colours from every side, so red, green and blue
    // stay recognisable as X, Y and Z (lit, their shaded faces came out almost black).
    axes.arrow({1, 0, 0}, 2.0f, 0.06f, QColor(220, 40, 40));
    axes.arrow({0, 1, 0}, 2.0f, 0.06f, QColor(40, 170, 60));
    axes.arrow({0, 0, 1}, 2.0f, 0.06f, QColor(40, 90, 220));
    m_axes = addMesh(axes, false);

    ui::translated(this, [this, extent, ground, north, east, other] {
        setLabels({{{0, extent + 0.3f, ground}, tr("N"), north, -1, true},
                   {{extent + 0.3f, 0, ground}, tr("E"), east, -1, true},
                   {{0, -extent * 0.7f - 0.3f, ground}, tr("S"), other, -1, false},
                   {{-extent * 0.7f - 0.3f, 0, ground}, tr("W"), other, -1, false},
                   {{2.25f, 0, 0}, QStringLiteral("X"), QColor(220, 40, 40), m_axes, true},
                   {{0, 2.25f, 0}, QStringLiteral("Y"), QColor(40, 170, 60), m_axes, true},
                   {{0, 0, 2.25f}, QStringLiteral("Z"), QColor(40, 90, 220), m_axes, true}});
    });
    updateModels();
}

void BoardView::setAttitude(const Eigen::Vector4d &q) {
    m_q = q;
    updateModels();
}

void BoardView::updateModels() {
    const QMatrix4x4 model = toQMatrix(bodyToWorld(m_q));
    setMeshModel(m_board, model);
    setMeshModel(m_axes, model);
}

} // namespace ins10
