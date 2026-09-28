#include "IQWidgets/igQtInterpolationFitWidget.h"

#include <QFont>
#include <QHBoxLayout>
#include <QLabel>
#include <QMouseEvent>
#include <QPainter>
#include <QPen>
#include <QPolygonF>
#include <QPushButton>
#include <QSpinBox>
#include <QVBoxLayout>
#include <QWheelEvent>

#include <IQWidgets/igQtCharts.h>
#include <iGameFlatArray.h>

#include <algorithm>
#include <cmath>

namespace {

/** 控制点命中半径（像素） */
constexpr double kHitRadiusPx = 10.0;

/** 背景 / 绘图区 / 网格 / 坐标轴颜色 */
const QColor kBackground(24, 26, 33);
const QColor kPlotArea(32, 35, 44);
const QColor kGridLine(56, 60, 72);
const QColor kAxisLine(130, 138, 155);
const QColor kTickText(150, 158, 175);
const QColor kHintText(190, 196, 208);
const QColor kPolyColor(235, 120, 60);     ///< 多项式插值曲线
const QColor kSplineColor(80, 210, 225);   ///< 分段三次样条曲线
const QColor kPointColor(232, 236, 244);   ///< 普通控制点
const QColor kHoverColor(255, 214, 102);   ///< hover 控制点
const QColor kSelectedColor(255, 170, 40); ///< 选中控制点

/** 默认控制点（和 Examples/Games102/TestHW1Interpolation.cpp 一致） */
std::vector<Eigen::Vector2f> DefaultPoints() {
    return {Eigen::Vector2f(-3.f, 1.2f), Eigen::Vector2f(-1.5f, 2.6f),
            Eigen::Vector2f(0.f, 0.8f), Eigen::Vector2f(1.5f, -1.f),
            Eigen::Vector2f(3.f, 0.4f)};
}

} // namespace

/* ================================================================== */
/* 画布                                                                */
/* ================================================================== */

igQtInterpolationCanvas::igQtInterpolationCanvas(QWidget* parent) : QWidget(parent) {
    setMouseTracking(true);
    setFocusPolicy(Qt::StrongFocus);
    setMinimumSize(minimumSizeHint());

    m_poly = iGame::HW1::New();
    m_spline = iGame::HW1::New();
    m_points = DefaultPoints();

    Refit();
    UpdateViewRange();
}

void igQtInterpolationCanvas::SetInterpType(iGame::HW1::InterpType type) {
    m_showSpline = (type == iGame::HW1::InterpType::CubicSpline);
    Refit();
    update();
    emit PlotChanged();
}

iGame::HW1::InterpType igQtInterpolationCanvas::GetInterpType() const {
    return m_showSpline ? iGame::HW1::InterpType::CubicSpline : iGame::HW1::InterpType::Polynomial;
}

QString igQtInterpolationCanvas::GetInterpName() const {
    return m_showSpline ? QStringLiteral("分段三次样条") : QStringLiteral("多项式插值");
}

void igQtInterpolationCanvas::SetSampleCount(int n) {
    if (n < 2) { n = 2; }
    if (n == m_samples) { return; }
    m_samples = n;
    Refit();
    update();
    emit PlotChanged();
}

void igQtInterpolationCanvas::ResetPoints() {
    m_points = DefaultPoints();
    m_selected = -1;
    m_hover = -1;
    Refit();
    UpdateViewRange();
    update();
    emit PlotChanged();
}

bool igQtInterpolationCanvas::Refit() {
    if (!m_poly || !m_spline) { return false; }

    m_poly->setpoints(m_points);
    m_poly->setnumber(m_samples);
    m_poly->setInterpType(iGame::HW1::InterpType::Polynomial);

    m_spline->setpoints(m_points);
    m_spline->setnumber(m_samples);
    m_spline->setInterpType(iGame::HW1::InterpType::CubicSpline);

    const bool okPoly = m_poly->Execute();
    const bool okSpline = m_spline->Execute();
    return okPoly && okSpline;
}

const iGame::HW1* igQtInterpolationCanvas::MainFit() const {
    return m_showSpline ? m_spline.operator->() : m_poly.operator->();
}

const iGame::HW1* igQtInterpolationCanvas::SecondFit() const {
    return m_showSpline ? m_poly.operator->() : m_spline.operator->();
}

float igQtInterpolationCanvas::GetMaxError() const {
    const iGame::HW1* fit = MainFit();
    return (fit == nullptr) ? 0.f : fit->getmaxerror();
}

std::vector<float> igQtInterpolationCanvas::SampleCurveY() const {
    std::vector<float> result;
    const iGame::HW1* fit = MainFit();
    if (fit == nullptr) { return result; }
    const auto& curve = fit->getcurve();
    result.reserve(curve.size());
    for (const auto& p : curve) { result.push_back(p.y()); }
    return result;
}

void igQtInterpolationCanvas::UpdateViewRange() {
    if (m_points.empty()) { return; }

    float x0 = m_points.front().x();
    float x1 = m_points.back().x();
    float lo = m_points.front().y();
    float hi = m_points.front().y();
    for (const auto& p : m_points) {
        lo = std::min(lo, p.y());
        hi = std::max(hi, p.y());
    }
    const iGame::HW1* fits[2] = {m_poly.operator->(), m_spline.operator->()};
    for (const iGame::HW1* fit : fits) {
        if (fit == nullptr) { continue; }
        for (const auto& p : fit->getcurve()) {
            lo = std::min(lo, p.y());
            hi = std::max(hi, p.y());
        }
    }

    if (x1 - x0 < 1e-3f) {
        x0 -= 1.f;
        x1 += 1.f;
    }
    const float padX = 0.12f * (x1 - x0);
    const float padY = 0.20f * std::max(hi - lo, 1e-3f);
    m_xmin = x0 - padX;
    m_xmax = x1 + padX;
    m_ymin = lo - padY;
    m_ymax = hi + padY;
}

QRectF igQtInterpolationCanvas::PlotRect() const {
    return QRectF(m_margin, m_margin, std::max(1.0, width() - 2.0 * m_margin),
                  std::max(1.0, height() - 2.0 * m_margin));
}

QPointF igQtInterpolationCanvas::DataToPixel(float x, float y) const {
    const QRectF r = PlotRect();
    const double sx = r.left() + (x - m_xmin) / (m_xmax - m_xmin) * r.width();
    // 屏幕 y 向下，数据 y 向上，所以这里翻转
    const double sy = r.bottom() - (y - m_ymin) / (m_ymax - m_ymin) * r.height();
    return QPointF(sx, sy);
}

void igQtInterpolationCanvas::PixelToData(const QPointF& pos, float& x, float& y) const {
    const QRectF r = PlotRect();
    x = m_xmin + static_cast<float>((pos.x() - r.left()) / r.width()) * (m_xmax - m_xmin);
    y = m_ymin + static_cast<float>((r.bottom() - pos.y()) / r.height()) * (m_ymax - m_ymin);
}

int igQtInterpolationCanvas::HitTest(const QPointF& pos, double thresholdPx) const {
    int best = -1;
    double bestDist = thresholdPx;
    for (int i = 0; i < static_cast<int>(m_points.size()); ++i) {
        const QPointF s = DataToPixel(m_points[i].x(), m_points[i].y());
        const double d = std::hypot(s.x() - pos.x(), s.y() - pos.y());
        if (d < bestDist) {
            bestDist = d;
            best = i;
        }
    }
    return best;
}

int igQtInterpolationCanvas::FindPointIndex(float x, float y) const {
    for (int i = 0; i < static_cast<int>(m_points.size()); ++i) {
        if (std::fabs(m_points[i].x() - x) < 1e-6f && std::fabs(m_points[i].y() - y) < 1e-6f) {
            return i;
        }
    }
    return -1;
}

void igQtInterpolationCanvas::paintEvent(QPaintEvent* event) {
    Q_UNUSED(event);

    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing, true);

    const QRectF plot = PlotRect();

    // 背景与绘图区
    painter.fillRect(rect(), kBackground);
    painter.fillRect(plot, kPlotArea);

    // 网格 10 x 6
    painter.setPen(QPen(kGridLine, 1.0));
    for (int i = 1; i < 10; ++i) {
        const double x = plot.left() + plot.width() * i / 10.0;
        painter.drawLine(QPointF(x, plot.top()), QPointF(x, plot.bottom()));
    }
    for (int i = 1; i < 6; ++i) {
        const double y = plot.top() + plot.height() * i / 6.0;
        painter.drawLine(QPointF(plot.left(), y), QPointF(plot.right(), y));
    }

    // 坐标轴（x 轴 y=0、y 轴 x=0）
    painter.setPen(QPen(kAxisLine, 2.0));
    if (m_ymin < 0.f && m_ymax > 0.f) {
        const double y0 = DataToPixel(0.f, 0.f).y();
        painter.drawLine(QPointF(plot.left(), y0), QPointF(plot.right(), y0));
    }
    if (m_xmin < 0.f && m_xmax > 0.f) {
        const double x0 = DataToPixel(0.f, 0.f).x();
        painter.drawLine(QPointF(x0, plot.top()), QPointF(x0, plot.bottom()));
    }

    // 刻度数值
    painter.setPen(QPen(kTickText));
    painter.setFont(QFont(QStringLiteral("Consolas"), 8));
    for (int i = 0; i <= 10; i += 2) {
        const double xv = m_xmin + (m_xmax - m_xmin) * i / 10.0;
        const double px = plot.left() + plot.width() * i / 10.0;
        painter.drawText(QPointF(px + 3.0, plot.bottom() - 4.0), QString::number(xv, 'g', 3));
    }
    for (int i = 0; i <= 6; i += 2) {
        const double yv = m_ymin + (m_ymax - m_ymin) * i / 6.0;
        const double py = plot.bottom() - plot.height() * i / 6.0;
        painter.drawText(QPointF(plot.left() + 4.0, py - 3.0), QString::number(yv, 'g', 3));
    }

    // 曲线绘制：另一条做对比（细、暗），主曲线（粗、亮）
    auto drawCurve = [&](const iGame::HW1* fit, const QColor& color, double width) {
        if (fit == nullptr) { return; }
        const auto& curve = fit->getcurve();
        if (curve.size() < 2) { return; }
        QPolygonF poly;
        poly.reserve(static_cast<int>(curve.size()));
        for (const auto& p : curve) { poly.append(DataToPixel(p.x(), p.y())); }
        painter.setPen(QPen(color, width));
        painter.setBrush(Qt::NoBrush);
        painter.drawPolyline(poly);
    };

    drawCurve(SecondFit(), kSplineColor.darker(180), 1.6);
    drawCurve(MainFit(), m_showSpline ? kSplineColor : kPolyColor, 2.6);

    // 曲线名称（画在曲线末端上方，方便区分）
    painter.setFont(QFont(QStringLiteral("Microsoft YaHei"), 8));
    auto drawLabel = [&](const iGame::HW1* fit, const QColor& color, const QString& name) {
        if (fit == nullptr || fit->getcurve().empty()) { return; }
        const auto& last = fit->getcurve().back();
        const QPointF s = DataToPixel(last.x(), last.y());
        painter.setPen(QPen(color));
        painter.drawText(QPointF(s.x() - 96.0, s.y() - 8.0), name);
    };
    drawLabel(m_spline.operator->(), kSplineColor, QStringLiteral("CubicSpline"));
    drawLabel(m_poly.operator->(), kPolyColor, QStringLiteral("Polynomial"));

    // 控制点
    for (int i = 0; i < static_cast<int>(m_points.size()); ++i) {
        if (i == m_selected) { continue; }
        const QPointF s = DataToPixel(m_points[i].x(), m_points[i].y());
        painter.setPen(QPen(QColor(20, 22, 28), 1.0));
        painter.setBrush(i == m_hover ? kHoverColor : kPointColor);
        painter.drawEllipse(s, 4.5, 4.5);
    }
    if (m_selected >= 0 && m_selected < static_cast<int>(m_points.size())) {
        const QPointF s = DataToPixel(m_points[m_selected].x(), m_points[m_selected].y());
        painter.setPen(QPen(QColor(20, 22, 28), 1.5));
        painter.setBrush(kSelectedColor);
        painter.drawEllipse(s, 7.0, 7.0);
    }

    // 左上角提示
    painter.setPen(QPen(kHintText));
    painter.setFont(QFont(QStringLiteral("Microsoft YaHei"), 8));
    painter.drawText(QRectF(plot.left() + 8.0, plot.top() + 6.0, plot.width() - 16.0, 20.0),
                     Qt::AlignLeft | Qt::AlignVCenter,
                     QStringLiteral("左键空白处新增点 / 左键拖动点 / 右键点删除 / 右键空白切换类型 / 滚轮缩放"));

    // 边框
    painter.setPen(QPen(kAxisLine, 1.0));
    painter.setBrush(Qt::NoBrush);
    painter.drawRect(plot);
}

void igQtInterpolationCanvas::mousePressEvent(QMouseEvent* event) {
    const QPointF pos = event->pos();

    if (event->button() == Qt::LeftButton) {
        const int hit = HitTest(pos, kHitRadiusPx);
        if (hit >= 0) {
            m_selected = hit;
            m_dragging = true;
        } else if (PlotRect().contains(pos)) {
            float x = 0.f;
            float y = 0.f;
            PixelToData(pos, x, y);
            m_points.emplace_back(x, y);
            std::sort(m_points.begin(), m_points.end(),
                      [](const Eigen::Vector2f& a, const Eigen::Vector2f& b) {
                          return a.x() < b.x();
                      });
            m_selected = FindPointIndex(x, y);
            m_dragging = true;
            Refit();
            // 视窗在松手时再重算，避免拖动过程中画面跳动
        }
    } else if (event->button() == Qt::RightButton) {
        const int hit = HitTest(pos, kHitRadiusPx);
        if (hit >= 0) {
            if (m_points.size() > 2) {
                m_points.erase(m_points.begin() + hit);
                m_selected = -1;
                m_hover = -1;
                Refit();
                UpdateViewRange();
            }
        } else {
            m_showSpline = !m_showSpline;
            Refit();
        }
    }

    update();
    emit PlotChanged();
}

void igQtInterpolationCanvas::mouseMoveEvent(QMouseEvent* event) {
    const QPointF pos = event->pos();

    if (m_dragging && m_selected >= 0 && m_selected < static_cast<int>(m_points.size())) {
        float x = 0.f;
        float y = 0.f;
        PixelToData(pos, x, y);

        // 把 x 夹在左右邻点之间：保证控制点按 x 递增，插值求解稳定
        const int i = m_selected;
        const float eps = 1e-3f * std::max(1.f, m_xmax - m_xmin);
        if (i > 0) { x = std::max(x, m_points[i - 1].x() + eps); }
        if (i + 1 < static_cast<int>(m_points.size())) {
            x = std::min(x, m_points[i + 1].x() - eps);
        }

        m_points[i] = Eigen::Vector2f(x, y);
        Refit();
        update();
        emit PlotChanged();
        return;
    }

    // hover 高亮
    const int hit = HitTest(pos, kHitRadiusPx);
    if (hit != m_hover) {
        m_hover = hit;
        update();
    }
}

void igQtInterpolationCanvas::mouseReleaseEvent(QMouseEvent* event) {
    Q_UNUSED(event);
    if (!m_dragging) { return; }
    m_dragging = false;
    Refit();
    UpdateViewRange();
    update();
    emit PlotChanged();
}

void igQtInterpolationCanvas::wheelEvent(QWheelEvent* event) {
    const double k = (event->delta() > 0) ? 0.9 : 1.0 / 0.9;
    const float cx = 0.5f * (m_xmin + m_xmax);
    const float cy = 0.5f * (m_ymin + m_ymax);
    m_xmin = cx + static_cast<float>((m_xmin - cx) * k);
    m_xmax = cx + static_cast<float>((m_xmax - cx) * k);
    m_ymin = cy + static_cast<float>((m_ymin - cy) * k);
    m_ymax = cy + static_cast<float>((m_ymax - cy) * k);
    update();
    event->accept();
}

/* ================================================================== */
/* 对话框                                                              */
/* ================================================================== */

igQtInterpolationFitWidget::igQtInterpolationFitWidget(QWidget* parent) : QDialog(parent) {
    setWindowTitle(QStringLiteral("hw1 插值型拟合函数图像"));
    resize(980, 660);
    setStyleSheet(QStringLiteral(
            "QDialog { background-color: #1F1F1F; }"
            "QLabel { color: #D5D8E0; }"
            "QPushButton { color: #E6E8EE; background-color: #2D2D30; border: 1px solid #3C3C3C;"
            "              border-radius: 4px; padding: 4px 10px; }"
            "QPushButton:hover { background-color: #3A3A3E; }"
            "QPushButton:pressed { background-color: #454549; }"
            "QSpinBox { color: #E6E8EE; background-color: #2D2D30; border: 1px solid #3C3C3C;"
            "           border-radius: 4px; padding: 2px 6px; }"));

    auto* mainLayout = new QVBoxLayout(this);
    mainLayout->setContentsMargins(10, 10, 10, 8);
    mainLayout->setSpacing(8);

    // ---- 工具条 ----
    auto* toolBar = new QHBoxLayout();
    toolBar->setSpacing(8);

    m_modeButton = new QPushButton(this);
    connect(m_modeButton, &QPushButton::clicked, this, [this]() {
        m_canvas->SetInterpType(m_canvas->GetInterpType() == iGame::HW1::InterpType::Polynomial
                                        ? iGame::HW1::InterpType::CubicSpline
                                        : iGame::HW1::InterpType::Polynomial);
        UpdateModeButton();
        UpdateStatus();
    });
    toolBar->addWidget(m_modeButton);

    toolBar->addWidget(new QLabel(QStringLiteral("采样点数"), this));
    m_sampleSpin = new QSpinBox(this);
    m_sampleSpin->setRange(16, 4000);
    m_sampleSpin->setSingleStep(16);
    m_sampleSpin->setValue(400);
    connect(m_sampleSpin, QOverload<int>::of(&QSpinBox::valueChanged), this, [this](int value) {
        m_canvas->SetSampleCount(value);
        UpdateStatus();
    });
    toolBar->addWidget(m_sampleSpin);

    m_resetButton = new QPushButton(QStringLiteral("重置控制点"), this);
    connect(m_resetButton, &QPushButton::clicked, this, [this]() {
        m_canvas->ResetPoints();
        UpdateStatus();
    });
    toolBar->addWidget(m_resetButton);

    toolBar->addStretch(1);

    m_chartButton = new QPushButton(QStringLiteral("导出为折线图"), this);
    connect(m_chartButton, &QPushButton::clicked, this, &igQtInterpolationFitWidget::ExportLineChart);
    toolBar->addWidget(m_chartButton);

    mainLayout->addLayout(toolBar);

    // ---- 画布 ----
    m_canvas = new igQtInterpolationCanvas(this);
    mainLayout->addWidget(m_canvas, 1);

    // ---- 状态栏 ----
    m_statusLabel = new QLabel(this);
    mainLayout->addWidget(m_statusLabel);

    connect(m_canvas, &igQtInterpolationCanvas::PlotChanged, this,
            &igQtInterpolationFitWidget::UpdateStatus);

    UpdateModeButton();
    UpdateStatus();
}

void igQtInterpolationFitWidget::UpdateModeButton() {
    if (m_modeButton == nullptr || m_canvas == nullptr) { return; }
    m_modeButton->setText(QStringLiteral("主曲线：%1（点击切换）").arg(m_canvas->GetInterpName()));
}

void igQtInterpolationFitWidget::UpdateStatus() {
    if (m_statusLabel == nullptr || m_canvas == nullptr) { return; }

    QString text = QStringLiteral("控制点 %1 个 | 主曲线 %2 | 采样 %3 点 | 过点最大偏差 %4")
                           .arg(m_canvas->GetPointCount())
                           .arg(m_canvas->GetInterpName())
                           .arg(m_canvas->GetSampleCount())
                           .arg(static_cast<double>(m_canvas->GetMaxError()), 0, 'g', 3);
    if (m_canvas->GetMaxError() > 1e-2f) {
        text += QStringLiteral(" | 提示：偏差偏大（多项式点数过多时会振荡），建议切到样条");
    }
    m_statusLabel->setText(text);
}

void igQtInterpolationFitWidget::ExportLineChart() {
    if (m_canvas == nullptr) { return; }

    auto values = iGame::FloatArray::New();
    const std::vector<float> samples = m_canvas->SampleCurveY();
    values->Reserve(static_cast<IGsize>(samples.size()));
    values->SetName((m_canvas->GetInterpName() + QStringLiteral(" 拟合曲线")).toStdString());
    for (float y : samples) { values->AddValue(y); }

    auto* chart = new igQtCharts(this);
    chart->setAttribute(Qt::WA_DeleteOnClose);
    chart->drawLineChart(values);
    chart->show();
}
