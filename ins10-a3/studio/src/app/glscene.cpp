#include "app/glscene.h"

#include <QMouseEvent>
#include <QOpenGLBuffer>
#include <QOpenGLContext>
#include <QOpenGLShaderProgram>
#include <QOpenGLVertexArrayObject>
#include <QPainter>
#include <QSurfaceFormat>
#include <QWheelEvent>

#include <cmath>

namespace ins10 {

// ---- MeshData ----------------------------------------------------------------------

void MeshData::vertex(const QVector3D &p, const QColor &c, const QVector3D &n) {
    vertices.insert(vertices.end(), {p.x(), p.y(), p.z(), static_cast<float>(c.redF()), static_cast<float>(c.greenF()),
                                     static_cast<float>(c.blueF()), n.x(), n.y(), n.z()});
}

void MeshData::line(const QVector3D &a, const QVector3D &b, const QColor &c) {
    vertex(a, c);
    vertex(b, c);
}

void MeshData::triangle(const QVector3D &a, const QVector3D &b, const QVector3D &c, const QColor &color) {
    const QVector3D n = QVector3D::normal(a, b, c);
    vertex(a, color, n);
    vertex(b, color, n);
    vertex(c, color, n);
}

void MeshData::quad(const QVector3D &a, const QVector3D &b, const QVector3D &c, const QVector3D &d, const QColor &color) {
    triangle(a, b, c, color);
    triangle(a, c, d, color);
}

void MeshData::box(const QVector3D &lo, const QVector3D &hi, const QColor &top, const QColor &bottom, const QColor &sides) {
    const QVector3D p000(lo.x(), lo.y(), lo.z()), p100(hi.x(), lo.y(), lo.z()), p010(lo.x(), hi.y(), lo.z()),
        p110(hi.x(), hi.y(), lo.z()), p001(lo.x(), lo.y(), hi.z()), p101(hi.x(), lo.y(), hi.z()),
        p011(lo.x(), hi.y(), hi.z()), p111(hi.x(), hi.y(), hi.z());
    quad(p001, p101, p111, p011, top);
    quad(p000, p010, p110, p100, bottom);
    quad(p000, p100, p101, p001, sides);
    quad(p100, p110, p111, p101, sides);
    quad(p110, p010, p011, p111, sides);
    quad(p010, p000, p001, p011, sides);
}

void MeshData::arrow(const QVector3D &axis, float length, float thickness, const QColor &color) {
    // Two perpendicular directions to build the square shaft and the head.
    QVector3D u = QVector3D::crossProduct(axis, std::abs(axis.z()) < 0.9f ? QVector3D(0, 0, 1) : QVector3D(1, 0, 0)).normalized();
    QVector3D v = QVector3D::crossProduct(axis, u).normalized();
    const float h = thickness * 0.5f;
    const float shaft = length * 0.8f;
    const QVector3D c0[4] = {(u + v) * h, (u - v) * h, (-u - v) * h, (-u + v) * h};
    for (int i = 0; i < 4; ++i) {
        const QVector3D a = c0[i], b = c0[(i + 1) % 4];
        quad(a, b, b + axis * shaft, a + axis * shaft, color);
    }
    const float r = thickness * 1.6f;
    const QVector3D base = axis * shaft, tip = axis * length;
    const QVector3D c1[4] = {base + (u + v) * r, base + (u - v) * r, base + (-u - v) * r, base + (-u + v) * r};
    for (int i = 0; i < 4; ++i) triangle(c1[i], c1[(i + 1) % 4], tip, color);
    quad(c1[3], c1[2], c1[1], c1[0], color);
}

// ---- GlScene -----------------------------------------------------------------------

struct GlScene::Mesh {
    MeshData data;
    bool lit = false;
    float pointSize = 3.0f;
    bool visible = true;
    bool dirty = true;
    QMatrix4x4 model;
    std::unique_ptr<QOpenGLVertexArrayObject> vao;
    std::unique_ptr<QOpenGLBuffer> vbo;
    int count = 0;
};

namespace {
const char *const kVertexShader = R"(#version 330 core
layout(location = 0) in vec3 position;
layout(location = 1) in vec3 color;
layout(location = 2) in vec3 normal;
uniform mat4 mvp;
uniform mat3 normalMatrix;
uniform bool lit;
uniform float pointSize;
out vec3 vColor;
void main() {
    gl_Position = mvp * vec4(position, 1.0);
    gl_PointSize = pointSize;
    vec3 c = color;
    if (lit && dot(normal, normal) > 0.0) {
        vec3 n = normalize(normalMatrix * normal);
        float d = max(dot(n, normalize(vec3(0.35, -0.45, 0.82))), 0.0);
        c = color * (0.42 + 0.58 * d);
    }
    vColor = c;
}
)";

const char *const kFragmentShader = R"(#version 330 core
in vec3 vColor;
uniform bool roundPoints;
out vec4 fragColor;
void main() {
    if (roundPoints) {
        vec2 d = gl_PointCoord - vec2(0.5);
        if (dot(d, d) > 0.25) discard;
    }
    fragColor = vec4(vColor, 1.0);
}
)";
} // namespace

GlScene::GlScene(QWidget *parent) : QOpenGLWidget(parent) {
    QSurfaceFormat format;
    format.setVersion(3, 3);
    format.setProfile(QSurfaceFormat::CoreProfile);
    format.setDepthBufferSize(24);
    format.setSamples(4);
    setFormat(format);
    setMinimumSize(200, 150);
}

GlScene::~GlScene() {
    if (!m_initialized || !context()) return;
    makeCurrent();
    for (auto &mesh : m_meshes) {
        mesh->vao.reset();
        mesh->vbo.reset();
    }
    m_program.reset();
    doneCurrent();
}

int GlScene::addMesh(const MeshData &meshData, bool lit, float pointSize) {
    auto mesh = std::make_unique<Mesh>();
    mesh->data = meshData;
    mesh->lit = lit;
    mesh->pointSize = pointSize;
    m_meshes.push_back(std::move(mesh));
    update();
    return static_cast<int>(m_meshes.size()) - 1;
}

void GlScene::setMeshData(int id, const MeshData &meshData) {
    if (id < 0 || id >= static_cast<int>(m_meshes.size())) return;
    m_meshes[static_cast<size_t>(id)]->data = meshData;
    m_meshes[static_cast<size_t>(id)]->dirty = true;
    update();
}

void GlScene::setMeshModel(int id, const QMatrix4x4 &model) {
    if (id < 0 || id >= static_cast<int>(m_meshes.size())) return;
    m_meshes[static_cast<size_t>(id)]->model = model;
    update();
}

void GlScene::setMeshVisible(int id, bool visible) {
    if (id < 0 || id >= static_cast<int>(m_meshes.size())) return;
    m_meshes[static_cast<size_t>(id)]->visible = visible;
    update();
}

void GlScene::setMeshPointSize(int id, float size) {
    if (id < 0 || id >= static_cast<int>(m_meshes.size())) return;
    m_meshes[static_cast<size_t>(id)]->pointSize = size;
    update();
}

void GlScene::setLabels(const QList<Label> &labels) {
    m_labels = labels;
    update();
}

void GlScene::setOrbit(float azimuthDeg, float elevationDeg, float distance) {
    m_azimuth = std::fmod(azimuthDeg, 360.0f);
    m_elevation = std::clamp(elevationDeg, -89.0f, 90.0f);
    m_distance = std::clamp(distance, 0.5f, 200.0f);
    update();
}

void GlScene::setTarget(const QVector3D &target) {
    m_target = target;
    update();
}

void GlScene::setDefaultView(float azimuthDeg, float elevationDeg, float distance) {
    m_defaultAzimuth = azimuthDeg;
    m_defaultElevation = elevationDeg;
    m_defaultDistance = distance;
    setOrbit(azimuthDeg, elevationDeg, distance);
}

void GlScene::topView() { setOrbit(-90.0f, 90.0f, m_distance); }

void GlScene::resetView() { setOrbit(m_defaultAzimuth, m_defaultElevation, m_defaultDistance); }

QMatrix4x4 GlScene::viewMatrix() const {
    const float az = qDegreesToRadians(m_azimuth), el = qDegreesToRadians(m_elevation);
    const QVector3D dir(std::cos(el) * std::cos(az), std::cos(el) * std::sin(az), std::sin(el));
    // The camera's own up direction: never parallel to the view, even from straight above.
    const QVector3D up(-std::cos(az) * std::sin(el), -std::sin(az) * std::sin(el), std::cos(el));
    QMatrix4x4 view;
    view.lookAt(m_target + dir * m_distance, m_target, up);
    return view;
}

QMatrix4x4 GlScene::projectionMatrix() const {
    QMatrix4x4 projection;
    projection.perspective(38.0f, width() / std::max(1.0f, static_cast<float>(height())), 0.05f, 500.0f);
    return projection;
}

QPointF GlScene::project(const QVector3D &world) const {
    const QVector4D clip = projectionMatrix() * viewMatrix() * QVector4D(world, 1.0f);
    if (clip.w() <= 1e-6f) return {-1e6, -1e6};
    const QVector3D ndc = clip.toVector3D() / clip.w();
    return {(ndc.x() * 0.5 + 0.5) * width(), (0.5 - ndc.y() * 0.5) * height()};
}

void GlScene::initializeGL() {
    m_initialized = true;
    QOpenGLContext *ctx = context();
    const QSurfaceFormat f = ctx ? ctx->format() : QSurfaceFormat();
    if (!ctx || !ctx->isValid() || ctx->isOpenGLES() || f.version() < qMakePair(3, 3)) {
        m_problem = tr("OpenGL 3.3 is not available here (got %1.%2). The 3D view is off; everything else works.")
                        .arg(f.majorVersion())
                        .arg(f.minorVersion());
        emit glReady(false);
        return;
    }
    initializeOpenGLFunctions();
    m_program = std::make_unique<QOpenGLShaderProgram>();
    if (!m_program->addShaderFromSourceCode(QOpenGLShader::Vertex, kVertexShader) ||
        !m_program->addShaderFromSourceCode(QOpenGLShader::Fragment, kFragmentShader) || !m_program->link()) {
        m_problem = tr("The 3D view's shaders did not compile: %1").arg(m_program->log());
        m_program.reset();
        emit glReady(false);
        return;
    }
    m_glOk = true;
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_PROGRAM_POINT_SIZE);
    glEnable(GL_MULTISAMPLE);
    emit glReady(true);
}

void GlScene::resizeGL(int, int) {}

void GlScene::upload(Mesh &mesh) {
    if (!mesh.vao) {
        mesh.vao = std::make_unique<QOpenGLVertexArrayObject>();
        mesh.vao->create();
        mesh.vbo = std::make_unique<QOpenGLBuffer>(QOpenGLBuffer::VertexBuffer);
        mesh.vbo->create();
        mesh.vbo->setUsagePattern(QOpenGLBuffer::DynamicDraw);
    }
    mesh.vao->bind();
    mesh.vbo->bind();
    mesh.vbo->allocate(mesh.data.vertices.data(), static_cast<int>(mesh.data.vertices.size() * sizeof(float)));
    const int stride = 9 * sizeof(float);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, stride, nullptr);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, stride, reinterpret_cast<const void *>(3 * sizeof(float)));
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, stride, reinterpret_cast<const void *>(6 * sizeof(float)));
    mesh.vao->release();
    mesh.count = mesh.data.vertexCount();
    mesh.dirty = false;
}

void GlScene::paintGL() {
    if (!m_glOk) {
        QPainter painter(this);
        painter.fillRect(rect(), palette().color(QPalette::Window));
        painter.setPen(palette().color(QPalette::WindowText));
        painter.drawText(rect().adjusted(20, 20, -20, -20), Qt::AlignCenter | Qt::TextWordWrap,
                         m_problem.isEmpty() ? tr("Starting OpenGL...") : m_problem);
        return;
    }
    const QColor bg = palette().color(QPalette::Base);
    glClearColor(static_cast<float>(bg.redF()), static_cast<float>(bg.greenF()), static_cast<float>(bg.blueF()), 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_PROGRAM_POINT_SIZE);
    const QMatrix4x4 viewProjection = projectionMatrix() * viewMatrix();
    m_program->bind();
    for (auto &mesh : m_meshes) {
        if (!mesh->visible) continue;
        if (mesh->dirty) upload(*mesh);
        if (mesh->count == 0) continue;
        m_program->setUniformValue("mvp", viewProjection * mesh->model);
        m_program->setUniformValue("normalMatrix", mesh->model.normalMatrix());
        m_program->setUniformValue("lit", mesh->lit);
        m_program->setUniformValue("pointSize", mesh->pointSize * static_cast<float>(devicePixelRatioF()));
        m_program->setUniformValue("roundPoints", mesh->data.kind == MeshData::Kind::Points);
        mesh->vao->bind();
        const GLenum mode = mesh->data.kind == MeshData::Kind::Lines    ? GL_LINES
                            : mesh->data.kind == MeshData::Kind::Points ? GL_POINTS
                                                                         : GL_TRIANGLES;
        glDrawArrays(mode, 0, mesh->count);
        mesh->vao->release();
    }
    m_program->release();
    glDisable(GL_DEPTH_TEST);

    // Labels on top, with QPainter after the GL drawing (it saves and restores the GL
    // state it touches).
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);
    for (const Label &label : m_labels) {
        QVector3D p = label.position;
        if (label.mesh >= 0 && label.mesh < static_cast<int>(m_meshes.size()))
            p = m_meshes[static_cast<size_t>(label.mesh)]->model.map(p);
        const QPointF s = project(p);
        if (s.x() < -1e5) continue;
        QFont labelFont = font();
        labelFont.setBold(label.bold);
        painter.setFont(labelFont);
        painter.setPen(label.color.isValid() ? label.color : palette().color(QPalette::Text));
        painter.drawText(QRectF(s.x() - 40, s.y() - 10, 80, 20), Qt::AlignCenter, label.text);
    }
}

void GlScene::mousePressEvent(QMouseEvent *event) { m_lastMouse = event->position(); }

void GlScene::mouseMoveEvent(QMouseEvent *event) {
    if (!(event->buttons() & (Qt::LeftButton | Qt::RightButton))) return;
    const QPointF d = event->position() - m_lastMouse;
    m_lastMouse = event->position();
    setOrbit(m_azimuth - static_cast<float>(d.x()) * 0.4f, m_elevation + static_cast<float>(d.y()) * 0.4f, m_distance);
}

void GlScene::wheelEvent(QWheelEvent *event) {
    setOrbit(m_azimuth, m_elevation, m_distance * std::pow(0.88f, event->angleDelta().y() / 120.0f));
    event->accept();
}

void GlScene::mouseDoubleClickEvent(QMouseEvent *) { resetView(); }

} // namespace ins10
