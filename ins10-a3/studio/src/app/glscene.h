// A small OpenGL 3.3 core scene for the 3D views: coloured meshes (triangles, lines,
// points) with optional flat lighting, an orbiting camera in an ENU world (z up), and
// text labels drawn with QPainter on top.
//
// When no OpenGL 3.3 context can be made (remote desktop, the "offscreen" platform),
// the widget says so instead of drawing; glAvailable() tells tests to skip.
#pragma once

#include <QColor>
#include <QMatrix4x4>
#include <QOpenGLExtraFunctions>
#include <QOpenGLWidget>
#include <QVector3D>

#include <memory>
#include <vector>

class QOpenGLShaderProgram;

namespace ins10 {

// Vertex data: position, colour, normal (zero normal: unlit).
struct MeshData {
    enum class Kind { Triangles, Lines, Points };
    Kind kind = Kind::Triangles;
    std::vector<float> vertices; // x y z r g b nx ny nz per vertex

    void vertex(const QVector3D &p, const QColor &c, const QVector3D &n = {});
    void line(const QVector3D &a, const QVector3D &b, const QColor &c);
    void triangle(const QVector3D &a, const QVector3D &b, const QVector3D &c, const QColor &color);
    void quad(const QVector3D &a, const QVector3D &b, const QVector3D &c, const QVector3D &d, const QColor &color);
    // Axis-aligned box from min to max corner; faces coloured top / bottom / sides.
    void box(const QVector3D &min, const QVector3D &max, const QColor &top, const QColor &bottom, const QColor &sides);
    // Arrow from the origin along `axis` (unit) of length `length`: square shaft and pyramid head.
    void arrow(const QVector3D &axis, float length, float thickness, const QColor &color);
    int vertexCount() const { return static_cast<int>(vertices.size() / 9); }
};

class GlScene : public QOpenGLWidget, protected QOpenGLExtraFunctions {
    Q_OBJECT
public:
    explicit GlScene(QWidget *parent = nullptr);
    ~GlScene() override;

    bool glAvailable() const { return m_glOk; }
    bool glInitialized() const { return m_initialized; }
    QString glProblem() const { return m_problem; }

    int addMesh(const MeshData &meshData, bool lit = false, float pointSize = 3.0f);
    void setMeshData(int id, const MeshData &meshData);
    void setMeshModel(int id, const QMatrix4x4 &model);
    void setMeshVisible(int id, bool visible);
    void setMeshPointSize(int id, float size);

    struct Label {
        QVector3D position;
        QString text;
        QColor color;
        int mesh = -1; // the position moves with this mesh's model matrix
        bool bold = false;
    };
    void setLabels(const QList<Label> &labels);
    const QList<Label> &labels() const { return m_labels; }

    void setOrbit(float azimuthDeg, float elevationDeg, float distance);
    void setTarget(const QVector3D &target);
    void topView();
    void resetView();
    float azimuth() const { return m_azimuth; }
    float elevation() const { return m_elevation; }
    float distance() const { return m_distance; }
    void setDefaultView(float azimuthDeg, float elevationDeg, float distance);

    // Projection of a world point to widget coordinates (for tests and labels).
    QPointF project(const QVector3D &world) const;

signals:
    void glReady(bool ok);

protected:
    void initializeGL() override;
    void resizeGL(int w, int h) override;
    void paintGL() override;
    void mousePressEvent(QMouseEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override;
    void wheelEvent(QWheelEvent *event) override;
    void mouseDoubleClickEvent(QMouseEvent *event) override;

private:
    struct Mesh;
    void upload(Mesh &mesh);
    QMatrix4x4 viewMatrix() const;
    QMatrix4x4 projectionMatrix() const;

    std::vector<std::unique_ptr<Mesh>> m_meshes;
    QList<Label> m_labels;
    std::unique_ptr<QOpenGLShaderProgram> m_program;
    bool m_glOk = false, m_initialized = false;
    QString m_problem;
    float m_azimuth = -120.0f, m_elevation = 25.0f, m_distance = 7.0f;
    float m_defaultAzimuth = -120.0f, m_defaultElevation = 25.0f, m_defaultDistance = 7.0f;
    QVector3D m_target;
    QPointF m_lastMouse;
};

} // namespace ins10
