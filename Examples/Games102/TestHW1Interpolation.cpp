/**
 * games102 HW1 —— 交互式「函数图像 + 可拖动控制点 + 插值型拟合」示例
 *
 * 画面内容（全部用 Painter2D 以像素坐标绘制，不依赖相机，坐标系原点在左下、y 向上）：
 *   - 深色底板 + 网格 + 坐标轴（x 轴 y=0、y 轴 x=0）
 *   - 两条插值曲线：多项式插值（橙红，主）与分段三次样条（青色，备选），拖动时实时重算
 *   - 控制点：左键在图上空白处单击**新增**一个点并立即进入拖动；在已有点附近按下则**选中**并拖动
 *   - 右键控制点**删除**；右键空白处**切换**主曲线（多项式 <-> 样条）；滚轮缩放视窗
 *
 * 运行：
 *   testHW1Interpolation            交互窗口（关窗退出）
 *   testHW1Interpolation --spline   以样条为主曲线启动
 *   testHW1Interpolation --test     无窗口自检：校验“曲线严格过控制点”等，返回 0/1（ctest 用）
 *
 * 拖点实现说明：iGameCore 里没有“给任意 2D 图层拾取点”的接口，
 * `Painter2D` 只画不拾取（图元没有命中查询）。所以这里用一个自定义 InteractorStyle
 * 接管鼠标（屏幕空间命中判定 10px），并把 Interactor 的内部样式换成
 * “点集只有一个远点的拖拽样式”——它的拾取半径由点集包围盒推出，此处恒为 0，
 * 于是永远不会拾取到东西，左键拖动也就不会再旋转相机，鼠标完全归本示例所有。
 */

#include <games102/hw1.h>

#include <iGameBrush.h>
#include <iGameCamera.h>
#include <iGameInteractor.h>
#include <iGameInteractorStyle.h>
#include <iGamePainter2D.h>
#include <iGamePen.h>
#include <iGamePoints.h>
#include <iGameRenderWindow.h>
#include <iGameScene.h>
#include <iGameSelection.h>
#include <iGameTextOverlay2DActor.h>
#include <iGameType.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <iostream>
#include <string>
#include <vector>

IGAME_NAMESPACE_BEGIN
namespace {

/* ==================== 画布与坐标映射 ==================== */

constexpr float kHitRadiusPx = 10.f;   ///< 控制点命中半径（像素）
constexpr int kSamples = 240;          ///< 曲线采样点数

/** 绘制区尺寸（framebuffer 像素，与 Painter2D 内部使用的 GL viewport 一致） */
struct Canvas {
    float w{1280.f};
    float h{720.f};
    float mouseScale{1.f};   ///< 鼠标事件坐标(逻辑窗口) → framebuffer 像素 的比例
};

Canvas ReadCanvas(Interactor* interactor) {
    Canvas canvas;
    if (interactor == nullptr) { return canvas; }
    Camera* camera = interactor->GetCamera();
    if (camera == nullptr) { return canvas; }

    const igm::uvec2 logical = camera->GetViewPort();
    const igm::uvec2 scaled = camera->GetScaledViewPort();
    if (scaled.x > 0 && scaled.y > 0) {
        canvas.w = static_cast<float>(scaled.x);
        canvas.h = static_cast<float>(scaled.y);
    } else if (logical.x > 0 && logical.y > 0) {
        canvas.w = static_cast<float>(logical.x);
        canvas.h = static_cast<float>(logical.y);
    }
    canvas.mouseScale = (logical.x > 0) ? static_cast<float>(scaled.x) / static_cast<float>(logical.x)
                                        : 1.f;
    if (canvas.mouseScale <= 0.f) { canvas.mouseScale = 1.f; }
    return canvas;
}

/** 像素坐标（y 向上，与 Painter2D 一致） */
struct Px {
    float x{0.f};
    float y{0.f};
};

Vector2ui ToUi(const Px& p) {
    const float x = (p.x < 0.f) ? 0.f : p.x;
    const float y = (p.y < 0.f) ? 0.f : p.y;
    return Vector2ui{static_cast<uint32_t>(x + 0.5f), static_cast<uint32_t>(y + 0.5f)};
}

/* ==================== 函数图像状态 ==================== */

struct Plot {
    std::vector<Eigen::Vector2f> ctrl;   ///< 控制点（数据坐标，x 严格递增）
    HW1::Pointer poly;                   ///< 多项式插值
    HW1::Pointer spline;                 ///< 分段三次样条

    float xmin{-1.f}, xmax{1.f}, ymin{-1.f}, ymax{1.f};   ///< 数据视窗
    float margin{70.f};                                   ///< 绘图区留边（像素）

    int selected{-1};      ///< 选中的控制点下标
    int hover{-1};         ///< hover 的控制点下标
    bool dragging{false};  ///< 是否正在拖动
    bool showSpline{false};///< true = 以样条为主曲线

    /* ---- 数据 <-> 像素 ---- */
    Px ToPixel(const Canvas& c, float x, float y) const {
        const float left = margin, right = c.w - margin;
        const float bottom = margin, top = c.h - margin;
        return Px{left + (x - xmin) / (xmax - xmin) * (right - left),
                  bottom + (y - ymin) / (ymax - ymin) * (top - bottom)};
    }
    void ToData(const Canvas& c, const Px& p, float& x, float& y) const {
        const float left = margin, right = c.w - margin;
        const float bottom = margin, top = c.h - margin;
        x = xmin + (p.x - left) / (right - left) * (xmax - xmin);
        y = ymin + (p.y - bottom) / (top - bottom) * (ymax - ymin);
    }
    bool InPlotArea(const Canvas& c, const Px& p) const {
        return p.x >= margin && p.x <= c.w - margin && p.y >= margin && p.y <= c.h - margin;
    }

    /** 屏幕空间最近控制点（阈值像素） */
    int HitTest(const Canvas& c, const Px& p, float threshold) const {
        int best = -1;
        float bestDist = threshold;
        for (int i = 0; i < static_cast<int>(ctrl.size()); ++i) {
            const Px s = ToPixel(c, ctrl[i].x(), ctrl[i].y());
            const float d = std::sqrt((s.x - p.x) * (s.x - p.x) + (s.y - p.y) * (s.y - p.y));
            if (d < bestDist) {
                bestDist = d;
                best = i;
            }
        }
        return best;
    }

    /** 重新求解两条曲线（拖动时每次鼠标移动都会调用） */
    bool Refit() {
        if (!poly || !spline) { return false; }
        poly->setpoints(ctrl);
        poly->setnumber(kSamples);
        poly->setInterpType(HW1::InterpType::Polynomial);
        spline->setpoints(ctrl);
        spline->setnumber(kSamples);
        spline->setInterpType(HW1::InterpType::CubicSpline);
        const bool okPoly = poly->Execute();
        const bool okSpline = spline->Execute();
        return okPoly && okSpline;
    }

    /** 按控制点与曲线自动取视窗范围（留边距），避免曲线贴边或出界 */
    void FitView() {
        if (ctrl.empty()) { return; }
        float x0 = ctrl.front().x();
        float x1 = ctrl.back().x();
        float lo = ctrl.front().y();
        float hi = ctrl.front().y();
        for (const auto& p : ctrl) {
            lo = std::min(lo, p.y());
            hi = std::max(hi, p.y());
        }
        if (poly) {
            for (const auto& p : poly->getcurve()) {
                lo = std::min(lo, p.y());
                hi = std::max(hi, p.y());
            }
        }
        if (spline) {
            for (const auto& p : spline->getcurve()) {
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
        xmin = x0 - padX;
        xmax = x1 + padX;
        ymin = lo - padY;
        ymax = hi + padY;
    }

    /** 主曲线（用于状态栏） */
    const HW1* Active() const { return showSpline ? spline.operator->() : poly.operator->(); }
};

/* ==================== 绘制 ==================== */

/** PainterBase::Draw() 按 penWidth 升序分组渲染，且同组内先线后三角形：
 *  用笔宽做图层 —— 底板(1) → 网格(2) → 坐标轴(3) → 备选曲线(4) → 主曲线(5) → 控制点(6)。 */
constexpr float kLayerBack = 1.f;
constexpr float kLayerGrid = 2.f;
constexpr float kLayerAxis = 3.f;
constexpr float kLayerSecond = 4.f;
constexpr float kLayerMain = 5.f;
constexpr float kLayerPoint = 6.f;

void DrawCurve(Painter2D* painter, const Plot& plot, const Canvas& canvas, const HW1* fit,
               float layer, int r, int g, int b) {
    if (fit == nullptr) { return; }
    const auto& curve = fit->getcurve();
    if (curve.size() < 2) { return; }

    painter->SetPen(Pen::Style::SolidLine);
    painter->SetPen(r, g, b);
    painter->SetPen(layer);
    for (size_t i = 0; i + 1 < curve.size(); ++i) {
        painter->DrawLine(ToUi(plot.ToPixel(canvas, curve[i].x(), curve[i].y())),
                          ToUi(plot.ToPixel(canvas, curve[i + 1].x(), curve[i + 1].y())));
    }
}

void DrawPlot(Painter2D* painter, const Plot& plot, const Canvas& canvas) {
    if (painter == nullptr) { return; }
    painter->Clear();
    if (plot.ctrl.empty()) { return; }

    const float left = plot.margin;
    const float right = canvas.w - plot.margin;
    const float bottom = plot.margin;
    const float top = canvas.h - plot.margin;

    /* ---- 1) 底板 ---- */
    painter->SetPen(Pen::Style::NoPen);
    painter->SetBrush(Brush::Style::SolidPattern);
    painter->SetBrush(24, 26, 33);
    painter->SetPen(kLayerBack);
    painter->DrawRect(ToUi(Px{left, bottom}), ToUi(Px{right, top}));

    /* ---- 2) 网格（10 x 6 等分）---- */
    painter->SetPen(Pen::Style::SolidLine);
    painter->SetPen(46, 50, 60);
    painter->SetPen(kLayerGrid);
    for (int i = 1; i < 10; ++i) {
        const float x = left + (right - left) * static_cast<float>(i) / 10.f;
        painter->DrawLine(ToUi(Px{x, bottom}), ToUi(Px{x, top}));
    }
    for (int i = 1; i < 6; ++i) {
        const float y = bottom + (top - bottom) * static_cast<float>(i) / 6.f;
        painter->DrawLine(ToUi(Px{left, y}), ToUi(Px{right, y}));
    }

    /* ---- 3) 坐标轴（x 轴 y=0、y 轴 x=0）---- */
    painter->SetPen(Pen::Style::SolidLine);
    painter->SetPen(120, 128, 145);
    painter->SetPen(kLayerAxis);
    if (plot.ymin < 0.f && plot.ymax > 0.f) {
        const float y0 = plot.ToPixel(canvas, 0.f, 0.f).y;
        painter->DrawLine(ToUi(Px{left, y0}), ToUi(Px{right, y0}));
    }
    if (plot.xmin < 0.f && plot.xmax > 0.f) {
        const float x0 = plot.ToPixel(canvas, 0.f, 0.f).x;
        painter->DrawLine(ToUi(Px{x0, bottom}), ToUi(Px{x0, top}));
    }

    /* ---- 4/5) 两条插值曲线：主曲线更亮更粗，另一条做对比 ---- */
    const HW1* mainCurve = plot.showSpline ? plot.spline.operator->() : plot.poly.operator->();
    const HW1* secondCurve = plot.showSpline ? plot.poly.operator->() : plot.spline.operator->();
    DrawCurve(painter, plot, canvas, secondCurve, kLayerSecond, 96, 128, 140);
    DrawCurve(painter, plot, canvas, mainCurve, kLayerMain,
              plot.showSpline ? 80 : 235, plot.showSpline ? 210 : 120, plot.showSpline ? 225 : 60);

    /* ---- 6) 控制点（实心圆，选中的最后画所以压在最上面）---- */
    painter->SetPen(Pen::Style::NoPen);
    painter->SetBrush(Brush::Style::SolidPattern);
    painter->SetPen(kLayerPoint);
    for (int i = 0; i < static_cast<int>(plot.ctrl.size()); ++i) {
        if (i == plot.selected) { continue; }
        const Px s = plot.ToPixel(canvas, plot.ctrl[i].x(), plot.ctrl[i].y());
        if (i == plot.hover) { painter->SetBrush(255, 214, 102); }
        else { painter->SetBrush(230, 234, 242); }
        painter->DrawCircle(ToUi(s), 4.5, 20);
    }
    if (plot.selected >= 0 && plot.selected < static_cast<int>(plot.ctrl.size())) {
        const Px s = plot.ToPixel(canvas, plot.ctrl[plot.selected].x(), plot.ctrl[plot.selected].y());
        painter->SetBrush(255, 214, 102);
        painter->DrawCircle(ToUi(s), 7.0, 24);
    }
}

void UpdateOverlay(const Plot& plot, TextOverlay2DActor* overlay) {
    if (overlay == nullptr) { return; }
    std::string text = "HW1 interpolation | main: ";
    text += plot.showSpline ? "CubicSpline" : "Polynomial";
    text += " | points: ";
    text += std::to_string(plot.ctrl.size());
    if (plot.poly) {
        text += " | polyErr: ";
        text += std::to_string(plot.poly->getmaxerror());
    }
    if (plot.spline) {
        text += " | splineErr: ";
        text += std::to_string(plot.spline->getmaxerror());
    }
    text += "  [L-click: add/drag  R-click point: del  R-click empty: switch  wheel: zoom]";
    overlay->SetText(text);
    overlay->SetAnchorToBottomRight(false);
    overlay->SetPosition(12.f, 10.f);
    overlay->SetScale(0.10f);
    overlay->SetColor(igm::vec3{0.86f, 0.88f, 0.93f});
    overlay->SetVisible(true);
}

/* ==================== 鼠标交互 ==================== */

class PlotStyle : public InteractorStyle {
public:
    I_OBJECT(PlotStyle);
    static Pointer New() { return new PlotStyle; }

    void Initialize(SmartPointer<Interactor> interactor) override { m_Interactor = interactor; }

    void SetPlot(Plot* plot) { m_Plot = plot; }
    void SetPainter(SmartPointer<Painter2D> painter) { m_Painter = painter; }
    void SetOverlay(SmartPointer<TextOverlay2DActor> overlay) { m_Overlay = overlay; }

    /** 重新绘制整幅图像并刷新状态栏（由外部在初始化后调用一次，之后由鼠标事件驱动） */
    void Redraw() {
        if (m_Plot == nullptr) { return; }
        const Canvas canvas = ReadCanvas(m_Interactor.operator->());
        DrawPlot(m_Painter.operator->(), *m_Plot, canvas);
        UpdateOverlay(*m_Plot, m_Overlay.operator->());
    }

    void MousePressEvent(IEvent event) override {
        if (m_Plot == nullptr || m_Plot->ctrl.empty()) { return; }
        const Canvas canvas = ReadCanvas(m_Interactor.operator->());
        const Px p{event.pos.x * canvas.mouseScale, canvas.h - event.pos.y * canvas.mouseScale};

        if (event.button == MouseButton::LeftButton) {
            const int hit = m_Plot->HitTest(canvas, p, kHitRadiusPx);
            if (hit >= 0) {
                // 选中已有点 → 开始拖动
                m_Plot->selected = hit;
                m_Plot->dragging = true;
            } else if (m_Plot->InPlotArea(canvas, p)) {
                // 图内空白处 → 新增控制点并立即拖动
                float dx = 0.f, dy = 0.f;
                m_Plot->ToData(canvas, p, dx, dy);
                m_Plot->ctrl.push_back(Eigen::Vector2f(dx, dy));
                std::sort(m_Plot->ctrl.begin(), m_Plot->ctrl.end(),
                          [](const Eigen::Vector2f& a, const Eigen::Vector2f& b) {
                              return a.x() < b.x();
                          });
                // 排序后找回刚加入的点
                int idx = 0;
                for (int i = 0; i < static_cast<int>(m_Plot->ctrl.size()); ++i) {
                    if (std::fabs(m_Plot->ctrl[i].x() - dx) < 1e-6f &&
                        std::fabs(m_Plot->ctrl[i].y() - dy) < 1e-6f) {
                        idx = i;
                        break;
                    }
                }
                m_Plot->selected = idx;
                m_Plot->dragging = true;
                m_Plot->Refit();
            }
        } else if (event.button == MouseButton::RightButton) {
            const int hit = m_Plot->HitTest(canvas, p, kHitRadiusPx);
            if (hit >= 0) {
                // 删除控制点（至少保留 2 个，否则无法插值）
                if (m_Plot->ctrl.size() > 2) {
                    m_Plot->ctrl.erase(m_Plot->ctrl.begin() + hit);
                    m_Plot->selected = -1;
                    m_Plot->Refit();
                    m_Plot->FitView();
                }
            } else {
                // 空白处右键 → 切换主曲线
                m_Plot->showSpline = !m_Plot->showSpline;
                m_Plot->Refit();
            }
        }
        Redraw();
    }

    void MouseMoveEvent(IEvent event) override {
        if (m_Plot == nullptr || m_Plot->ctrl.empty()) { return; }
        const Canvas canvas = ReadCanvas(m_Interactor.operator->());
        const Px p{event.pos.x * canvas.mouseScale, canvas.h - event.pos.y * canvas.mouseScale};

        if (m_Plot->dragging && m_Plot->selected >= 0 &&
            m_Plot->selected < static_cast<int>(m_Plot->ctrl.size())) {
            float dx = 0.f, dy = 0.f;
            m_Plot->ToData(canvas, p, dx, dy);

            // x 夹在左右邻点之间：保证控制点始终按 x 递增，插值求解稳定
            const int i = m_Plot->selected;
            const float eps = 1e-3f * std::max(1.f, m_Plot->xmax - m_Plot->xmin);
            if (i > 0) { dx = std::max(dx, m_Plot->ctrl[i - 1].x() + eps); }
            if (i + 1 < static_cast<int>(m_Plot->ctrl.size())) {
                dx = std::min(dx, m_Plot->ctrl[i + 1].x() - eps);
            }
            m_Plot->ctrl[i] = Eigen::Vector2f(dx, dy);
            m_Plot->Refit();
            Redraw();
            return;
        }

        // hover 高亮
        const int hit = m_Plot->HitTest(canvas, p, kHitRadiusPx);
        if (hit != m_Plot->hover) {
            m_Plot->hover = hit;
            Redraw();
        }
    }

    void MouseReleaseEvent(IEvent event) override {
        if (m_Plot == nullptr) { return; }
        if (m_Plot->dragging) {
            m_Plot->dragging = false;
            m_Plot->Refit();
            m_Plot->FitView();   // 松手后再重算视窗，避免拖动过程中画面乱跳
            Redraw();
        }
    }

    void WheelEvent(IEvent event) override {
        if (m_Plot == nullptr) { return; }
        // 以视窗中心为锚点缩放
        const float k = (event.delta > 0.0) ? 0.9f : 1.f / 0.9f;
        const float cx = 0.5f * (m_Plot->xmin + m_Plot->xmax);
        const float cy = 0.5f * (m_Plot->ymin + m_Plot->ymax);
        m_Plot->xmin = cx + (m_Plot->xmin - cx) * k;
        m_Plot->xmax = cx + (m_Plot->xmax - cx) * k;
        m_Plot->ymin = cy + (m_Plot->ymin - cy) * k;
        m_Plot->ymax = cy + (m_Plot->ymax - cy) * k;
        Redraw();
    }

protected:
    PlotStyle() = default;   // I_OBJECT 声明了拷贝/移动构造，隐式默认构造不再生成

private:
    Plot* m_Plot{nullptr};
    SmartPointer<Painter2D> m_Painter;
    SmartPointer<TextOverlay2DActor> m_Overlay;
    SmartPointer<Interactor> m_Interactor;
};

/* ==================== 无窗口自检 ==================== */

int CheckOne(const std::vector<Eigen::Vector2f>& pts, HW1::InterpType type, const char* name,
             int samples, int& failures) {
    auto fit = HW1::New();
    fit->setpoints(pts);
    fit->setnumber(samples);
    fit->setInterpType(type);

    if (!fit->Execute()) {
        std::cout << "FAIL [" << name << "] Execute() 返回 false: " << fit->getmessage() << "\n";
        ++failures;
        return 0;
    }
    std::cout << "[" << name << "] points=" << fit->getpoints().size()
              << " samples=" << fit->getcurve().size()
              << " maxError=" << fit->getmaxerror();
    if (!fit->getmessage().empty()) { std::cout << " message=\"" << fit->getmessage() << "\""; }
    std::cout << "\n";

    if (static_cast<int>(fit->getcurve().size()) != fit->getnumber()) {
        std::cout << "  FAIL: 采样点数 != setnumber()\n";
        ++failures;
    }
    // 注意：x 重复的输入在 setpoints() 里会被排序去重，所以按“有效控制点”校验
    const auto& valid = fit->getpoints();
    for (const auto& p : valid) {
        if (std::fabs(fit->hanshu(p.x()) - p.y()) > 1e-3f) {
            std::cout << "  FAIL: x=" << p.x() << " 处未过点（f=" << fit->hanshu(p.x())
                      << " 期望 " << p.y() << "）\n";
            ++failures;
        }
    }
    if (valid.size() != pts.size()) {
        std::cout << "  note: 输入 " << pts.size() << " 个点，x 去重后 " << valid.size() << " 个\n";
    }
    if (fit->getmaxerror() > 1e-3f) {
        std::cout << "  FAIL: 过点最大偏差过大\n";
        ++failures;
    }

    // 采样点单调递增的 x（绘制折线的前提）
    const auto& curve = fit->getcurve();
    for (size_t i = 1; i < curve.size(); ++i) {
        if (curve[i].x() <= curve[i - 1].x()) {
            std::cout << "  FAIL: 采样 x 非递增\n";
            ++failures;
            break;
        }
    }
    // 打印少量采样值便于人工核对
    std::cout << "  sample:";
    for (int i = 0; i < 5; ++i) {
        const auto& c = curve[static_cast<size_t>(i) * (curve.size() - 1) / 4];
        std::cout << " (" << c.x() << "," << c.y() << ")";
    }
    std::cout << "\n";
    return 0;
}

int RunSelfTest() {
    int failures = 0;

    // 1) 五个控制点：多项式(4 次)与自然三次样条都应严格过点
    const std::vector<Eigen::Vector2f> demo{
            {-3.f, 1.2f}, {-1.5f, 2.6f}, {0.f, 0.8f}, {1.5f, -1.f}, {3.f, 0.4f}};
    CheckOne(demo, HW1::InterpType::Polynomial, "Polynomial/5pts", 240, failures);
    CheckOne(demo, HW1::InterpType::CubicSpline, "CubicSpline/5pts", 240, failures);

    // 2) 两点：退化为直线，两种插值都应过点
    const std::vector<Eigen::Vector2f> two{{-1.f, -2.f}, {2.f, 4.f}};
    CheckOne(two, HW1::InterpType::Polynomial, "Polynomial/2pts", 16, failures);
    CheckOne(two, HW1::InterpType::CubicSpline, "CubicSpline/2pts", 16, failures);

    // 3) 乱序 + x 重复的输入：应自动排序去重后仍过点
    const std::vector<Eigen::Vector2f> messy{
            {2.f, 0.5f}, {-1.f, 1.f}, {0.5f, -0.25f}, {0.5f, 9.f}, {-2.f, 0.f}};
    CheckOne(messy, HW1::InterpType::CubicSpline, "CubicSpline/messy", 64, failures);

    // 4) 多项式系数个数 = 控制点数，且 hanshu 与采样一致
    {
        auto fit = HW1::New();
        fit->setpoints(demo);
        fit->setnumber(32);
        fit->setInterpType(HW1::InterpType::Polynomial);
        fit->Execute();
        if (static_cast<int>(fit->getalpha().size()) != static_cast<int>(fit->getpoints().size())) {
            std::cout << "FAIL: 多项式系数个数 != 控制点数\n";
            ++failures;
        }
        if (std::fabs(fit->hanshu(1.25f) - fit->hanshu(1.25f, HW1::InterpType::Polynomial)) > 1e-6f) {
            std::cout << "FAIL: hanshu(x) 与 hanshu(x, Polynomial) 不一致\n";
            ++failures;
        }
    }

    // 5) 单点：输出常值曲线
    {
        auto fit = HW1::New();
        fit->setpoints({{1.f, 3.f}});
        fit->setnumber(8);
        if (!fit->Execute() || fit->getcurve().size() != 8) {
            std::cout << "FAIL: 单点控制应输出常值曲线\n";
            ++failures;
        }
    }

    std::cout << (failures == 0 ? "RESULT: PASS\n" : "RESULT: FAIL\n");
    return failures == 0 ? 0 : 1;
}

} // namespace
IGAME_NAMESPACE_END

int main(int argc, char** argv) {
    bool selfTest = false;
    bool startSpline = false;
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--test") == 0) { selfTest = true; }
        else if (std::strcmp(argv[i], "--spline") == 0) { startSpline = true; }
        else if (std::strcmp(argv[i], "--help") == 0 || std::strcmp(argv[i], "-h") == 0) {
            std::cout << "usage: testHW1Interpolation [--test] [--spline]\n"
                         "  --test    无窗口自检（多项式/样条是否严格过控制点等）\n"
                         "  --spline  以分段三次样条为主曲线启动\n";
            return 0;
        }
    }
    if (selfTest) { return iGame::RunSelfTest(); }

    /* ---------- 场景 / 窗口 ---------- */
    auto scene = iGame::Scene::New();

    auto window = iGame::RenderWindow::New();
    window->SetSize(1280, 720);
    window->SetTitle("games102 HW1 - interpolation fit: L add/drag, R del/switch, wheel zoom");
    window->SetScene(scene);

    auto interactor = iGame::Interactor::New();
    interactor->Initialize(scene);   // 内部会建默认 BasicStyle（左键拖拽=旋转相机）

    // 用“只有一个远点的点集”的拖拽样式替换内部样式：拖拽拾取半径由点集包围盒得出，
    // 这里恒为 0 → 永远不会拾取到点 → 左键拖动不再旋转相机，鼠标交给 PlotStyle。
    auto neutralPoints = iGame::Points::New();
    neutralPoints->AddPoint(0.f, 0.f, 0.f);
    auto neutralSelection = iGame::Selection::New();
    neutralSelection->SetPoints(neutralPoints);
    interactor->RequestDragPointStyle(neutralSelection);

    /* ---------- 图像状态与交互样式 ---------- */
    iGame::Plot plot;
    plot.poly = iGame::HW1::New();
    plot.spline = iGame::HW1::New();
    plot.showSpline = startSpline;

    // 初始控制点（可随意拖着改；x 会被自动排序）
    plot.ctrl = {{-3.f, 1.2f}, {-1.5f, 2.6f}, {0.f, 0.8f}, {1.5f, -1.f}, {3.f, 0.4f}};

    auto style = iGame::PlotStyle::New();
    style->SetPlot(&plot);
    style->SetPainter(scene->GetPainter2D());
    style->SetOverlay(scene->GetTextOverlay2DActor());
    style->Initialize(interactor);
    interactor->_SetSpecialInteractor("HW1PlotStyle", style);

    window->SetInteractor(interactor);

    /* ---------- 首次求解并绘制 ---------- */
    plot.Refit();
    plot.FitView();
    style->Redraw();

    std::cout << "games102 HW1 - interactive interpolation fit\n"
                 "  左键点图上空白处 : 新增控制点（并立即进入拖动）\n"
                 "  左键点控制点附近 : 选中并拖动（实时重拟合；x 被夹在左右邻点之间）\n"
                 "  右键点控制点     : 删除该点（至少保留 2 个）\n"
                 "  右键点空白处     : 切换主曲线（Polynomial <-> CubicSpline）\n"
                 "  滚轮             : 缩放视窗；关闭窗口退出\n"
                 "  提示：点数超过 16 时整体多项式会剧烈振荡，可切到样条对比\n";
    window->Show();
    return 0;
}
