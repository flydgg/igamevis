#ifndef iGameHW1_h
#define iGameHW1_h

#include "iGameFilter.h"

#include <Eigen/Dense>

#include <string>
#include <vector>

IGAME_NAMESPACE_BEGIN

/**
 * games102 HW1 —— 插值型拟合（interpolation fitting）
 *
 * 给定一组控制点 (x_i, y_i)（x 互不相同），生成一条**严格经过每个控制点**的曲线：
 *
 *  - InterpType::Polynomial ：整体多项式插值。解范德蒙德方程组 V·α = y
 *    得到系数 α（α[i] 是 x^i 的系数，degree = n-1），求值时用 Horner 秦九韶法
 *    y = Σ α_i x^i。点数较多时会出现龙格（Runge）振荡。
 *  - InterpType::CubicSpline：分段三次样条（自然边界 M_0 = M_{n-1} = 0），
 *    C² 连续；节点多也稳定，适合交互式拖点的场景。
 *
 * 结果按 num 个采样点均匀输出（getcurve() 给 (x, y) 便于绘制函数图像，
 * getout() 保留为原始作业接口的“只要 y 值”版本）。
 */
class HW1 : public Filter {
public:
    I_OBJECT(HW1);
    static Pointer New() { return new HW1; }

    enum class InterpType {
        Polynomial,  ///< 整体多项式插值（拉格朗日 / 范德蒙德解系数）
        CubicSpline  ///< 分段三次样条插值（自然边界）
    };

    /* ---------------- 输入 ---------------- */

    /**
     * @brief 设置控制点。
     * @note 内部会按 x 升序排序，并去掉 x 重复（间隔 < 1e-6）的点。
     */
    void setpoints(const std::vector<Eigen::Vector2f>& point);

    /** @brief 取排序去重后的控制点 */
    const std::vector<Eigen::Vector2f>& getpoints() const { return m_Points; }

    /** @brief 曲线采样点数（最小 2），默认 200 */
    void setnumber(int n);

    /** @brief 取采样点数 */
    int getnumber() const { return m_Num; }

    /** @brief 设置插值类型 */
    void setInterpType(InterpType type) { m_Type = type; }

    /** @brief 取插值类型 */
    InterpType getInterpType() const { return m_Type; }

    /** @brief 当前是否样条 */
    bool isSpline() const { return m_Type == InterpType::CubicSpline; }

    /* ---------------- 求解 / 求值 ---------------- */

    /**
     * @brief 多项式插值：解 V·α = y，返回系数 α（α[i] 对应 x^i）。
     * @param point 控制点
     * @return 系数数组；控制点为空时返回空数组并写入 getmessage()
     */
    std::vector<float> duoxiangshi(const std::vector<Eigen::Vector2f>& point);

    /** @brief 当前多项式系数 */
    const std::vector<float>& getalpha() const { return m_Alpha; }

    /** @brief 按当前插值类型求值 y = f(x) */
    float hanshu(float x);

    /** @brief 指定插值类型求值（不动内部状态，便于对比两条曲线） */
    float hanshu(float x, InterpType type);

    /* ---------------- 结果 ---------------- */

    /** @brief 计算采样曲线；控制点非法时返回 false 并写入 getmessage() */
    bool Execute() override;

    /** @brief 采样点的 y 值，长度 = getnumber()（保留原始作业接口） */
    std::vector<float> getout();

    /** @brief 采样曲线点 (x, y)，长度 = getnumber()，供绘制函数图像 */
    const std::vector<Eigen::Vector2f>& getcurve() const { return m_Curve; }

    /**
     * @brief 曲线在控制点处的最大偏差。
     * @note 插值型拟合应当严格过点，正常情况接近 0；数值病态（点太多 / x 太挤）
     *       时会变大，可用于自检。
     */
    float getmaxerror() const { return m_MaxError; }

    /** @brief 最近一次求解的提示信息（空串表示正常） */
    const std::string& getmessage() const { return m_Message; }

protected:
    HW1() {
        SetNumberOfInputs(1);
        SetNumberOfOutputs(1);
    }
    ~HW1() override = default;

    /** @brief 解多项式系数，失败返回 false */
    bool solvePolynomial();

    /** @brief 计算自然三次样条的二阶导 M_i，失败返回 false */
    bool buildSpline();

    /** @brief Horner 求值 */
    float evalPolynomial(float x) const;

    /** @brief 样条求值 */
    float evalSpline(float x) const;

    /** @brief 控制点落在的样条区间下标（x 在 [x_i, x_{i+1}] 内） */
    int findSegment(float x) const;

    std::vector<Eigen::Vector2f> m_Points;  ///< 控制点（排序去重后）
    int m_Num{200};                         ///< 采样点数
    InterpType m_Type{InterpType::Polynomial};

    std::vector<float> m_Alpha;             ///< 多项式系数 α
    std::vector<float> m_M;                 ///< 样条二阶导 M_i
    std::vector<float> m_Out;               ///< 采样 y 值
    std::vector<Eigen::Vector2f> m_Curve;   ///< 采样 (x, y)
    float m_MaxError{0.f};                  ///< 过点最大偏差
    std::string m_Message;                  ///< 提示信息
};

IGAME_NAMESPACE_END

#endif
