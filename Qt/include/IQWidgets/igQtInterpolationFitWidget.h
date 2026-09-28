/**
 * @class   igQtInterpolationFitWidget
 * @brief   games102 HW1 —— 交互式「插值型拟合函数图像」对话框。
 *
 * 画布用 QPainter 自绘（不依赖 OpenGL 视口 / 3D 相机），交互：
 *   - 左键在图上空白处单击：新增一个控制点并立即进入拖动
 *   - 左键在控制点附近按下：选中并拖动（实时重拟合；x 被夹在左右邻点之间保持有序）
 *   - 右键控制点：删除该点（至少保留 2 个）
 *   - 右键空白处：切换主曲线（多项式插值 <-> 分段三次样条插值）
 *   - 滚轮：缩放视窗
 *
 * 拟合计算复用 iGameCore 的 HW1 filter（iGameCore/Filters/games102/hw1.h）。
 */
#pragma once

#include <QDialog>
#include <QPointF>
#include <QRectF>
#include <QSize>
#include <QString>
#include <QWidget>

#include <Eigen/Dense>

#include <vector>

#include <games102/hw1.h>

class QLabel;
class QPushButton;
class QSpinBox;
class QMouseEvent;
class QPaintEvent;
class QWheelEvent;

/**
 * @brief 自绘的插值曲线画布：管理控制点、求解并绘制两条插值曲线。
 */
class igQtInterpolationCanvas : public QWidget {
    Q_OBJECT

public:
    explicit igQtInterpolationCanvas(QWidget* parent = nullptr);

    /** @brief 设置主曲线类型（会立即重拟合） */
    void SetInterpType(iGame::HW1::InterpType type);

    /** @brief 当前主曲线类型 */
    iGame::HW1::InterpType GetInterpType() const;

    /** @brief 曲线采样点数 */
    void SetSampleCount(int n);

    /** @brief 取采样点数 */
    int GetSampleCount() const { return m_samples; }

    /** @brief 恢复初始控制点 */
    void ResetPoints();

    /** @brief 按控制点与曲线重新计算视窗范围 */
    void UpdateViewRange();

    /** @brief 主曲线的 y 采样值（导出折线图用） */
    std::vector<float> SampleCurveY() const;

    /** @brief 主曲线在控制点处的最大偏差（插值型拟合应接近 0） */
    float GetMaxError() const;

    /** @brief 当前控制点个数 */
    int GetPointCount() const { return static_cast<int>(m_points.size()); }

    /** @brief 主曲线名称 */
    QString GetInterpName() const;

    QSize sizeHint() const override { return QSize(880, 520); }
    QSize minimumSizeHint() const override { return QSize(360, 240); }

signals:
    /** @brief 控制点 / 主曲线类型 / 采样点数变化后发出 */
    void PlotChanged();

protected:
    void paintEvent(QPaintEvent* event) override;
    void mousePressEvent(QMouseEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;
    void mouseReleaseEvent(QMouseEvent* event) override;
    void wheelEvent(QWheelEvent* event) override;

private:
    QRectF PlotRect() const;
    QPointF DataToPixel(float x, float y) const;
    void PixelToData(const QPointF& pos, float& x, float& y) const;
    int HitTest(const QPointF& pos, double thresholdPx) const;
    int FindPointIndex(float x, float y) const;
    bool Refit();
    const iGame::HW1* MainFit() const;
    const iGame::HW1* SecondFit() const;

    std::vector<Eigen::Vector2f> m_points;   ///< 控制点（x 递增）
    iGame::HW1::Pointer m_poly;              ///< 多项式插值
    iGame::HW1::Pointer m_spline;            ///< 分段三次样条

    int m_samples{400};        ///< 采样点数
    bool m_showSpline{false};  ///< 主曲线是否为样条
    int m_selected{-1};        ///< 选中的控制点
    int m_hover{-1};           ///< hover 的控制点
    bool m_dragging{false};    ///< 是否正在拖动

    float m_xmin{-1.f}, m_xmax{1.f}, m_ymin{-1.f}, m_ymax{1.f};  ///< 数据视窗
    float m_margin{56.f};                                        ///< 绘图区留边（像素）
};

/**
 * @brief 承载画布的对话框：顶部工具条（切换主曲线 / 采样点数 / 重置 / 导出折线图）+ 状态栏。
 */
class igQtInterpolationFitWidget : public QDialog {
    Q_OBJECT

public:
    explicit igQtInterpolationFitWidget(QWidget* parent = nullptr);

private:
    void UpdateStatus();
    void UpdateModeButton();
    void ExportLineChart();

    igQtInterpolationCanvas* m_canvas{nullptr};
    QPushButton* m_modeButton{nullptr};
    QPushButton* m_resetButton{nullptr};
    QPushButton* m_chartButton{nullptr};
    QLabel* m_statusLabel{nullptr};
    QSpinBox* m_sampleSpin{nullptr};
};
