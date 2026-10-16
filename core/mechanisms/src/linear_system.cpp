#include "wpiemu/core/mechanisms/linear_system.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace wpiemu::core::mechanisms {

Matrix::Matrix(int rows, int cols)
    : rows_(std::max(rows, 0)), cols_(std::max(cols, 0)),
      data_(static_cast<std::size_t>(rows_) * static_cast<std::size_t>(cols_), 0.0) {}

Matrix Matrix::identity(int n) {
    Matrix m(n, n);
    for (int i = 0; i < n; ++i) m(i, i) = 1.0;
    return m;
}

Matrix Matrix::operator*(const Matrix& rhs) const {
    Matrix out(rows_, rhs.cols_);
    if (cols_ != rhs.rows_) return out;
    for (int i = 0; i < rows_; ++i)
        for (int k = 0; k < cols_; ++k) {
            const double a = (*this)(i, k);
            for (int j = 0; j < rhs.cols_; ++j) out(i, j) += a * rhs(k, j);
        }
    return out;
}

Matrix Matrix::operator+(const Matrix& rhs) const {
    Matrix out(rows_, cols_);
    if (rows_ != rhs.rows_ || cols_ != rhs.cols_) return out;
    for (std::size_t i = 0; i < data_.size(); ++i) out.data_[i] = data_[i] + rhs.data_[i];
    return out;
}

Matrix Matrix::operator*(double s) const {
    Matrix out = *this;
    for (double& v : out.data_) v *= s;
    return out;
}

double Matrix::norm1() const {
    double best = 0.0;
    for (int j = 0; j < cols_; ++j) {
        double sum = 0.0;
        for (int i = 0; i < rows_; ++i) sum += std::abs((*this)(i, j));
        best = std::max(best, sum);
    }
    return best;
}

Matrix expm(const Matrix& m) {
    const int n = m.rows();
    if (n == 0 || m.cols() != n) return Matrix(n, n);
    int squarings = 0;
    const double norm = m.norm1();
    if (norm > 0.5) squarings = std::max(0, static_cast<int>(std::ceil(std::log2(norm / 0.5))));
    const Matrix scaled = m * std::ldexp(1.0, -squarings);

    // Taylor series; with ||scaled|| <= 0.5, 18 terms reach well below double epsilon.
    Matrix result = Matrix::identity(n);
    Matrix term = Matrix::identity(n);
    for (int k = 1; k <= 18; ++k) {
        term = (term * scaled) * (1.0 / static_cast<double>(k));
        result = result + term;
    }
    for (int i = 0; i < squarings; ++i) result = result * result;
    return result;
}

DiscreteSystem discretizeAB(const Matrix& a, const Matrix& b, double dtSeconds) {
    const int n = a.rows();
    const int m = b.cols();
    Matrix aug(n + m, n + m);
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) aug(i, j) = a(i, j) * dtSeconds;
        for (int j = 0; j < m; ++j) aug(i, n + j) = b(i, j) * dtSeconds;
    }
    const Matrix phi = expm(aug);
    DiscreteSystem out{Matrix(n, n), Matrix(n, m)};
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) out.Ad(i, j) = phi(i, j);
        for (int j = 0; j < m; ++j) out.Bd(i, j) = phi(i, n + j);
    }
    return out;
}

namespace {

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

struct Gearbox {
    double kt;  // N*m/A of the whole gearbox
    double r;   // ohm of the whole gearbox
    double kv;  // rad/s/V
};

Gearbox gearbox(const sdk::MotorSpec& motor, int numMotors) {
    // WPILib scales stall torque/current and free current by numMotors: Kt unchanged, R / n.
    const double n = static_cast<double>(numMotors);
    const double stallCurrent = motor.stallCurrentA * n;
    const double freeCurrent = motor.freeCurrentA * n;
    const double r = motor.nominalVoltageV / stallCurrent;
    return {motor.stallTorqueNm * n / stallCurrent, r,
            motor.freeSpeedRadPerSec / (motor.nominalVoltageV - r * freeCurrent)};
}

bool valid(const sdk::MotorSpec& motor, int numMotors, double a, double b) {
    return numMotors >= 1 && a > 0.0 && b > 0.0 && motor.stallCurrentA > 0.0 && motor.stallTorqueNm > 0.0 &&
           motor.freeSpeedRadPerSec > 0.0;
}

LinearSystem nanSystem(int states) {
    LinearSystem s{Matrix(states, states), Matrix(states, 1)};
    for (int i = 0; i < states; ++i) {
        s.B(i, 0) = kNaN;
        for (int j = 0; j < states; ++j) s.A(i, j) = kNaN;
    }
    return s;
}

}  // namespace

LinearSystem elevatorSystem(const sdk::MotorSpec& motor, int numMotors, double massKg, double radiusM,
                            double gearing) {
    if (!valid(motor, numMotors, massKg, radiusM) || gearing <= 0.0) return nanSystem(2);
    const Gearbox g = gearbox(motor, numMotors);
    LinearSystem s{Matrix(2, 2), Matrix(2, 1)};
    s.A(0, 1) = 1.0;
    s.A(1, 1) = -gearing * gearing * g.kt / (g.r * radiusM * radiusM * massKg * g.kv);
    s.B(1, 0) = gearing * g.kt / (g.r * radiusM * massKg);
    return s;
}

LinearSystem flywheelSystem(const sdk::MotorSpec& motor, int numMotors, double jKgM2, double gearing) {
    if (!valid(motor, numMotors, jKgM2, gearing)) return nanSystem(1);
    const Gearbox g = gearbox(motor, numMotors);
    LinearSystem s{Matrix(1, 1), Matrix(1, 1)};
    s.A(0, 0) = -gearing * gearing * g.kt / (g.kv * g.r * jKgM2);
    s.B(0, 0) = gearing * g.kt / (g.r * jKgM2);
    return s;
}

LinearSystem dcMotorSystem(const sdk::MotorSpec& motor, int numMotors, double jKgM2, double gearing) {
    if (!valid(motor, numMotors, jKgM2, gearing)) return nanSystem(2);
    const Gearbox g = gearbox(motor, numMotors);
    LinearSystem s{Matrix(2, 2), Matrix(2, 1)};
    s.A(0, 1) = 1.0;
    s.A(1, 1) = -gearing * gearing * g.kt / (g.kv * g.r * jKgM2);
    s.B(1, 0) = gearing * g.kt / (g.r * jKgM2);
    return s;
}

}  // namespace wpiemu::core::mechanisms
