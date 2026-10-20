#pragma once

// Small dense-matrix + linear-system toolkit mirroring the parts of WPILib's wpimath that the 1-DOF
// plants need: LinearSystemId (elevator / flywheel / DC motor / single-jointed arm) and the matrix
// exponential discretization (frc::DiscretizeAB). No external dependency; double precision.
//
// Equations are those of allwpilib 2026 `LinearSystemId.java` (see docs/reference/mechanisms.md):
//   elevator   A = [[0,1],[0,-G^2 Kt/(R r^2 m Kv)]]   B = [0, G Kt/(R r m)]
//   flywheel   A = [-G^2 Kt/(Kv R J)]                 B = [G Kt/(R J)]
//   dc motor / single-jointed arm
//              A = [[0,1],[0,-G^2 Kt/(Kv R J)]]       B = [0, G Kt/(R J)]
// where Kt, R, Kv are those of the WHOLE gearbox (`numMotors` identical motors), exactly like
// `DCMotor.withReduction`-style aggregation in WPILib: Kt = Kt1, R = R1 / n, Kv = Kv1.

#include <cstddef>
#include <vector>

#include "wpiemu/sdk/dynamics/motor_model.h"

namespace wpiemu::core::mechanisms {

// Row-major dense matrix. Dimensions are runtime values; element access is unchecked.
class Matrix {
public:
    Matrix() = default;
    Matrix(int rows, int cols);

    static Matrix identity(int n);

    int rows() const { return rows_; }
    int cols() const { return cols_; }
    double& operator()(int r, int c) { return data_[static_cast<std::size_t>(r * cols_ + c)]; }
    double operator()(int r, int c) const { return data_[static_cast<std::size_t>(r * cols_ + c)]; }

    Matrix operator*(const Matrix& rhs) const;
    Matrix operator+(const Matrix& rhs) const;
    Matrix operator*(double s) const;
    // Induced 1-norm (max absolute column sum).
    double norm1() const;

private:
    int rows_ = 0;
    int cols_ = 0;
    std::vector<double> data_;
};

// exp(M) by scaling-and-squaring with a Taylor series. M must be square.
Matrix expm(const Matrix& m);

// Continuous system x' = A x + B u (outputs are the state; D = 0, as in WPILib's plants).
struct LinearSystem {
    Matrix A;
    Matrix B;
};

struct DiscreteSystem {
    Matrix Ad;
    Matrix Bd;
};

// Zero-order-hold discretization via the augmented-matrix exponential (WPILib DiscretizeAB).
DiscreteSystem discretizeAB(const Matrix& a, const Matrix& b, double dtSeconds);

// ---- LinearSystemId equivalents ------------------------------------------------------------------
// `motor` is ONE motor; `numMotors` identical motors share the gearbox. `gearing` = motor : output
// (> 1 reduces speed), `radiusM` = drum radius, `massKg`/`jKgM2` = carriage mass / output inertia.
// Invalid (non-positive) inputs yield a system of NaNs, the caller validates beforehand.
LinearSystem elevatorSystem(const sdk::MotorSpec& motor, int numMotors, double massKg, double radiusM,
                            double gearing);
LinearSystem flywheelSystem(const sdk::MotorSpec& motor, int numMotors, double jKgM2, double gearing);
LinearSystem dcMotorSystem(const sdk::MotorSpec& motor, int numMotors, double jKgM2, double gearing);
inline LinearSystem singleJointedArmSystem(const sdk::MotorSpec& motor, int numMotors, double jKgM2,
                                           double gearing) {
    return dcMotorSystem(motor, numMotors, jKgM2, gearing);
}

}  // namespace wpiemu::core::mechanisms
