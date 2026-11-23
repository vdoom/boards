#include "app/sphereview.h"

#include <cmath>
#include <numbers>

namespace ins10 {

namespace {
constexpr int kBands = 6, kSectors = 12;
constexpr float kPi = std::numbers::pi_v<float>;

QVector3D onSphere(float z, float phi, float r) {
    const float rho = std::sqrt(std::max(0.0f, 1.0f - z * z));
    return QVector3D(rho * std::cos(phi), rho * std::sin(phi), z) * r;
}
} // namespace

SphereView::SphereView(QWidget *parent) : GlScene(parent) {
    setDefaultView(-60.0f, 25.0f, 4.6f);
    MeshData wire;
    wire.kind = MeshData::Kind::Lines;
    const QColor grid(150, 150, 150);
    // The cell boundaries: circles of constant z, meridians every 30 deg.
    for (int b = 1; b < kBands; ++b) {
        const float z = -1.0f + 2.0f * b / kBands;
        for (int k = 0; k < 72; ++k) wire.line(onSphere(z, 2 * kPi * k / 72, 1.0f), onSphere(z, 2 * kPi * (k + 1) / 72, 1.0f), grid);
    }
    for (int s = 0; s < kSectors; ++s) {
        const float phi = -kPi + 2 * kPi * s / kSectors;
        for (int k = 0; k < 36; ++k) wire.line(onSphere(-1.0f + 2.0f * k / 36, phi, 1.0f), onSphere(-1.0f + 2.0f * (k + 1) / 36, phi, 1.0f), grid);
    }
    // Axes through the sphere.
    wire.line({-1.3f, 0, 0}, {1.3f, 0, 0}, QColor(220, 40, 40));
    wire.line({0, -1.3f, 0}, {0, 1.3f, 0}, QColor(40, 170, 60));
    wire.line({0, 0, -1.3f}, {0, 0, 1.3f}, QColor(40, 90, 220));
    addMesh(wire);
    m_cells = addMesh(MeshData(), true);
    m_clouds[0] = addMesh(MeshData());
    m_clouds[1] = addMesh(MeshData());
    m_markers = addMesh(MeshData());
    showAxisLabels(true);
}

void SphereView::showAxisLabels(bool show) {
    if (!show) {
        setLabels({});
        return;
    }
    setLabels({{{1.42f, 0, 0}, QStringLiteral("+X"), QColor(220, 40, 40), -1, true},
               {{-1.42f, 0, 0}, QStringLiteral("-X"), QColor(220, 40, 40), -1, false},
               {{0, 1.42f, 0}, QStringLiteral("+Y"), QColor(40, 170, 60), -1, true},
               {{0, -1.42f, 0}, QStringLiteral("-Y"), QColor(40, 170, 60), -1, false},
               {{0, 0, 1.42f}, QStringLiteral("+Z"), QColor(40, 90, 220), -1, true},
               {{0, 0, -1.42f}, QStringLiteral("-Z"), QColor(40, 90, 220), -1, false}});
}

void SphereView::setCells(const std::vector<bool> &cells) {
    MeshData mesh;
    if (cells.size() == static_cast<size_t>(kBands * kSectors)) {
        const QColor visited(60, 170, 90), empty(205, 70, 60);
        const float r = 0.9f;
        const int sub = 4; // patches per cell edge, so the cells look round
        for (int b = 0; b < kBands; ++b) {
            for (int s = 0; s < kSectors; ++s) {
                const QColor color = cells[static_cast<size_t>(b * kSectors + s)] ? visited : empty;
                const float z0 = -1.0f + 2.0f * b / kBands, z1 = -1.0f + 2.0f * (b + 1) / kBands;
                const float p0 = -kPi + 2 * kPi * s / kSectors, p1 = -kPi + 2 * kPi * (s + 1) / kSectors;
                // A small gap between cells keeps them apart visually.
                const float gz = (z1 - z0) * 0.04f, gp = (p1 - p0) * 0.04f;
                for (int i = 0; i < sub; ++i) {
                    for (int j = 0; j < sub; ++j) {
                        const float za = z0 + gz + (z1 - z0 - 2 * gz) * i / sub, zb = z0 + gz + (z1 - z0 - 2 * gz) * (i + 1) / sub;
                        const float pa = p0 + gp + (p1 - p0 - 2 * gp) * j / sub, pb = p0 + gp + (p1 - p0 - 2 * gp) * (j + 1) / sub;
                        mesh.quad(onSphere(za, pa, r), onSphere(za, pb, r), onSphere(zb, pb, r), onSphere(zb, pa, r), color);
                    }
                }
            }
        }
    }
    setMeshData(m_cells, mesh);
}

void SphereView::setCloud(int index, const Eigen::MatrixX3d &points, const QColor &color, float size) {
    if (index < 0 || index > 1) return;
    MeshData mesh;
    mesh.kind = MeshData::Kind::Points;
    mesh.vertices.reserve(static_cast<size_t>(points.rows()) * 9);
    for (Eigen::Index i = 0; i < points.rows(); ++i) {
        mesh.vertex(QVector3D(static_cast<float>(points(i, 0)), static_cast<float>(points(i, 1)), static_cast<float>(points(i, 2))), color);
    }
    setMeshData(m_clouds[index], mesh);
    setMeshPointSize(m_clouds[index], size);
}

void SphereView::clearCloud(int index) {
    if (index < 0 || index > 1) return;
    setMeshData(m_clouds[index], MeshData());
}

void SphereView::setMarkers(const std::vector<Eigen::Vector3d> &points, const QColor &color) {
    MeshData mesh;
    for (const Eigen::Vector3d &p : points) {
        // A small cube per marker, visible from every side.
        const QVector3D c(static_cast<float>(p[0]), static_cast<float>(p[1]), static_cast<float>(p[2]));
        const float h = 0.05f;
        mesh.box(c - QVector3D(h, h, h), c + QVector3D(h, h, h), color, color.darker(130), color.darker(115));
    }
    setMeshData(m_markers, mesh);
}

} // namespace ins10
