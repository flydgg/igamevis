#include "hw1.h"

#include <algorithm>
#include <cmath>

IGAME_NAMESPACE_BEGIN

namespace {
/** 整体多项式插值超过这个点数时，范德蒙德方程组会非常病态，提示振荡 */
constexpr int kPolynomialWarnPoints = 16;
/** 判定 x 重复的阈值 */
constexpr float kDuplicateEps = 1e-6f;
} // namespace

/* ------------------------------------------------------------------ */
/* 控制点                                                              */
/* ------------------------------------------------------------------ */
void HW1::setpoints(const std::vector<Eigen::Vector2f>& point) {
    m_Points = point;

    // 1) 按 x 升序排序（多项式/样条都要求节点有序）
    std::sort(m_Points.begin(), m_Points.end(),
              [](const Eigen::Vector2f& a, const Eigen::Vector2f& b) {
                  return a.x() < b.x();
              });

    // 2) 去掉 x 重复的点，保留第一个（Vandermonde 要求 x 互不相同）
    std::vector<Eigen::Vector2f> unique;
    unique.reserve(m_Points.size());
    for (const auto& p : m_Points) {
        if (unique.empty() || std::fabs(p.x() - unique.back().x()) > kDuplicateEps) {
            unique.push_back(p);
        }
    }
    m_Points.swap(unique);
}

void HW1::setnumber(int n) { m_Num = (n < 2) ? 2 : n; }

/* ------------------------------------------------------------------ */
/* 多项式插值：解 V·α = y                                              */
/* ------------------------------------------------------------------ */
std::vector<float> HW1::duoxiangshi(const std::vector<Eigen::Vector2f>& point) {
    m_Alpha.clear();

    const int n = static_cast<int>(point.size());
    if (n <= 0) {
        m_Message = "控制点为空，无法做多项式插值";
        return m_Alpha;
    }

    // 范德蒙德矩阵 V(i, j) = x_i^j，右端项 y_i
    Eigen::MatrixXd V(n, n);
    Eigen::VectorXd y(n);
    for (int i = 0; i < n; ++i) {
        double xp = 1.0; // x^0, x^1, ... 逐次乘 x，避免反复调 pow
        for (int j = 0; j < n; ++j) {
            V(i, j) = xp;
            xp *= static_cast<double>(point[i].x());
        }
        y(i) = static_cast<double>(point[i].y());
    }

    // 选主元的 QR 分解比直接求逆稳定
    const Eigen::VectorXd a = V.colPivHouseholderQr().solve(y);

    m_Alpha.resize(n);
    for (int i = 0; i < n; ++i) { m_Alpha[i] = static_cast<float>(a(i)); }

    if (n > kPolynomialWarnPoints) {
        m_Message = "控制点较多，整体多项式插值会出现剧烈振荡，建议切换到分段三次样条";
    }
    return m_Alpha;
}

float HW1::evalPolynomial(float x) const {
    // Horner: ((α_n·x + α_{n-1})·x + ...)·x + α_0
    float y = 0.f;
    for (int i = static_cast<int>(m_Alpha.size()) - 1; i >= 0; --i) {
        y = y * x + m_Alpha[i];
    }
    return y;
}

/* ------------------------------------------------------------------ */
/* 分段三次样条（自然边界）                                             */
/* ------------------------------------------------------------------ */
bool HW1::buildSpline() {
    m_M.clear();

    const int n = static_cast<int>(m_Points.size());
    if (n < 2) { return false; }

    // 区间长度 h_i = x_{i+1} - x_i
    std::vector<double> h(n - 1);
    for (int i = 0; i + 1 < n; ++i) {
        h[i] = static_cast<double>(m_Points[i + 1].x() - m_Points[i].x());
    }

    // 自然边界：M_0 = M_{n-1} = 0
    m_M.assign(n, 0.f);
    if (n == 2) { return true; } // 两点退化为直线

    // 未知量 M_1 .. M_{n-2}，三对角方程组
    const int m = n - 2;
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(m, m);
    Eigen::VectorXd b(m);
    for (int i = 0; i < m; ++i) {
        const int k = i + 1; // 对应节点 k
        if (i > 0) { A(i, i - 1) = h[k - 1]; }
        A(i, i) = 2.0 * (h[k - 1] + h[k]);
        if (i + 1 < m) { A(i, i + 1) = h[k]; }

        const double dy1 = static_cast<double>(m_Points[k + 1].y() - m_Points[k].y());
        const double dy0 = static_cast<double>(m_Points[k].y() - m_Points[k - 1].y());
        b(i) = 6.0 * (dy1 / h[k] - dy0 / h[k - 1]);
    }

    const Eigen::VectorXd sol = A.colPivHouseholderQr().solve(b);
    for (int i = 0; i < m; ++i) { m_M[i + 1] = static_cast<float>(sol(i)); }
    return true;
}

int HW1::findSegment(float x) const {
    const int n = static_cast<int>(m_Points.size());
    int i = 0;
    while (i + 2 < n && x > m_Points[i + 1].x()) { ++i; }
    return i;
}

float HW1::evalSpline(float x) const {
    const int n = static_cast<int>(m_Points.size());
    if (n == 0) { return 0.f; }
    if (n == 1) { return m_Points[0].y(); }

    // 区间外按端点值处理（插值曲线只在 [x_0, x_{n-1}] 上有定义）
    if (x <= m_Points.front().x()) { return m_Points.front().y(); }
    if (x >= m_Points.back().x()) { return m_Points.back().y(); }

    const int i = findSegment(x);
    const float hi = m_Points[i + 1].x() - m_Points[i].x();
    const float dx = x - m_Points[i].x();

    // S_i(x) = y_i + b_i·dx + c_i·dx² + d_i·dx³
    const float b = (m_Points[i + 1].y() - m_Points[i].y()) / hi -
                    hi * (2.f * m_M[i] + m_M[i + 1]) / 6.f;
    const float c = m_M[i] / 2.f;
    const float d = (m_M[i + 1] - m_M[i]) / (6.f * hi);

    return m_Points[i].y() + b * dx + c * dx * dx + d * dx * dx * dx;
}

/* ------------------------------------------------------------------ */
/* 求值 / 执行                                                          */
/* ------------------------------------------------------------------ */
float HW1::hanshu(float x) { return hanshu(x, m_Type); }

float HW1::hanshu(float x, InterpType type) {
    if (type == InterpType::CubicSpline) { return evalSpline(x); }
    return evalPolynomial(x);
}

bool HW1::Execute() {
    m_Message.clear();
    m_MaxError = 0.f;
    m_Out.clear();
    m_Curve.clear();

    const int n = static_cast<int>(m_Points.size());
    if (n == 0) {
        m_Message = "控制点为空";
        return false;
    }

    // 只有一个点：输出常值曲线
    if (n == 1) {
        m_Out.assign(m_Num, m_Points[0].y());
        m_Curve.assign(m_Num, m_Points[0]);
        return true;
    }

    // 求解
    if (m_Type == InterpType::Polynomial) {
        if (duoxiangshi(m_Points).empty()) { return false; }
    } else {
        if (!buildSpline()) {
            m_Message = "样条求解失败";
            return false;
        }
    }

    // 在 [x_0, x_{n-1}] 上均匀采样
    const int samples = (m_Num < 2) ? 2 : m_Num;
    const float x0 = m_Points.front().x();
    const float x1 = m_Points.back().x();

    m_Out.resize(samples);
    m_Curve.resize(samples);
    for (int i = 0; i < samples; ++i) {
        const float t = static_cast<float>(i) / static_cast<float>(samples - 1);
        const float x = x0 + (x1 - x0) * t;
        const float y = hanshu(x);
        m_Out[i] = y;
        m_Curve[i] = Eigen::Vector2f(x, y);
    }

    // 自检：插值型拟合必须严格经过所有控制点
    for (const auto& p : m_Points) {
        m_MaxError = std::max(m_MaxError, std::fabs(hanshu(p.x()) - p.y()));
    }

    if (m_Message.empty() && m_MaxError > 1e-2f) {
        m_Message = "曲线未能严格过点（数值病态，可减少控制点或改用样条）";
    }
    return true;
}

std::vector<float> HW1::getout() { return m_Out; }

IGAME_NAMESPACE_END
