#include "wpiemu/core/vision/pose_solver.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace wpiemu::core::vision {

namespace {

constexpr double kInf = std::numeric_limits<double>::infinity();

// ---- dense solve (Gaussian elimination with partial pivoting) -----------------------------------

template <int N>
bool solveLinear(std::array<std::array<double, N>, N> a, std::array<double, N> b,
                 std::array<double, N>& x) {
    double scale = 0.0;
    for (const auto& row : a)
        for (const double v : row) scale = std::max(scale, std::abs(v));
    if (!(scale > 0.0) || !std::isfinite(scale)) return false;
    for (int col = 0; col < N; ++col) {
        int pivot = col;
        for (int r = col + 1; r < N; ++r)
            if (std::abs(a[r][col]) > std::abs(a[pivot][col])) pivot = r;
        if (!(std::abs(a[pivot][col]) > 1e-13 * scale)) return false;
        std::swap(a[col], a[pivot]);
        std::swap(b[col], b[pivot]);
        for (int r = col + 1; r < N; ++r) {
            const double f = a[r][col] / a[col][col];
            if (f == 0.0) continue;
            for (int c = col; c < N; ++c) a[r][c] -= f * a[col][c];
            b[r] -= f * b[col];
        }
    }
    for (int row = N - 1; row >= 0; --row) {
        double s = b[row];
        for (int c = row + 1; c < N; ++c) s -= a[row][c] * x[c];
        x[row] = s / a[row][row];
    }
    return true;
}

// ---- IPPE for a square --------------------------------------------------------------------------

// The model is the unit square (half-size 1) in the plane Z = 0 of a right-handed model frame whose
// X axis is the tag +Y (viewer's right), Y axis the tag +Z (up) and Z axis the tag +X (outward
// normal). Corners in the contract order BL, BR, TR, TL. OpenCV camera frame: x right, y down,
// z forward.
constexpr double kModel[4][2] = {{-1.0, -1.0}, {1.0, -1.0}, {1.0, 1.0}, {-1.0, 1.0}};

// model frame -> tag frame: model (X, Y, Z) = (tag y, tag z, tag x)
constexpr Mat3d kModelFromTag = Mat3d::fromRows({0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}, {1.0, 0.0, 0.0});
// OpenCV camera frame -> neutral camera frame (X forward, Y left, Z up)
constexpr Mat3d kNeutralFromCv =
    Mat3d::fromRows({0.0, 0.0, 1.0}, {-1.0, 0.0, 0.0}, {0.0, -1.0, 0.0});

struct CvPose {
    Mat3d R;  // model frame -> OpenCV camera frame
    Vec3d t;
};

// Homography H (H33 = 1) with (x, y, 1) ~ H (X, Y, 1) for the four correspondences.
bool unitSquareHomography(const std::array<std::array<double, 2>, 4>& image, Mat3d& H) {
    std::array<std::array<double, 8>, 8> a{};
    std::array<double, 8> b{};
    for (std::size_t i = 0; i < 4; ++i) {
        const double X = kModel[i][0], Y = kModel[i][1];
        const double x = image[i][0], y = image[i][1];
        a[2 * i] = {X, Y, 1.0, 0.0, 0.0, 0.0, -x * X, -x * Y};
        b[2 * i] = x;
        a[2 * i + 1] = {0.0, 0.0, 0.0, X, Y, 1.0, -y * X, -y * Y};
        b[2 * i + 1] = y;
    }
    std::array<double, 8> h{};
    if (!solveLinear<8>(a, b, h)) return false;
    H.m = {h[0], h[1], h[2], h[3], h[4], h[5], h[6], h[7], 1.0};
    return true;
}

// Translation minimising the object-space collinearity error for a given rotation (the step
// OpenCV's IPPE uses): (I - v v^T / v.v) (R U + t) = 0 for every corner.
bool translationForRotation(const Mat3d& R, const std::array<Vec3d, 4>& rays, Vec3d& t) {
    std::array<std::array<double, 3>, 3> a{};
    std::array<double, 3> rhs{};
    for (std::size_t i = 0; i < 4; ++i) {
        const Vec3d& v = rays[i];
        const double inv = 1.0 / dot(v, v);
        const Vec3d u = R * Vec3d{kModel[i][0], kModel[i][1], 0.0};
        const double vv[3] = {v.x, v.y, v.z};
        const double uu[3] = {u.x, u.y, u.z};
        const double vu = dot(v, u);
        for (int r = 0; r < 3; ++r) {
            for (int c = 0; c < 3; ++c) a[r][c] += (r == c ? 1.0 : 0.0) - vv[r] * vv[c] * inv;
            // -(I - V) R U
            rhs[r] -= uu[r] - vv[r] * vu * inv;
        }
    }
    std::array<double, 3> x{};
    if (!solveLinear<3>(a, rhs, x)) return false;
    t = {x[0], x[1], x[2]};
    return true;
}

// The two poses (unit model) a planar square can have for the normalised image points `rays`
// (x, y, 1).
bool ippeUnitSquare(const std::array<Vec3d, 4>& rays, std::array<CvPose, 2>& out) {
    std::array<std::array<double, 2>, 4> image{};
    for (std::size_t i = 0; i < 4; ++i) image[i] = {rays[i].x, rays[i].y};
    Mat3d H;
    if (!unitSquareHomography(image, H)) return false;

    // The image of the model origin and the 2x2 Jacobian of the homography there.
    const double p = H(0, 2), q = H(1, 2);
    const double j00 = H(0, 0) - H(2, 0) * p, j01 = H(0, 1) - H(2, 1) * p;
    const double j10 = H(1, 0) - H(2, 0) * q, j11 = H(1, 1) - H(2, 1) * q;

    // Rv rotates the optical axis onto the ray through the model origin, so that origin sits on
    // the axis of a virtual camera where the Jacobian is (up to scale) the upper-left block of the
    // rotation.
    const double nu = std::sqrt(p * p + q * q + 1.0);
    const Vec3d vhat{p / nu, q / nu, 1.0 / nu};
    const Mat3d K = Mat3d::skew(cross({0.0, 0.0, 1.0}, vhat));
    const Mat3d Rv = Mat3d{} + K + (K * K) * (1.0 / (1.0 + vhat.z));

    // J = W A / t_z for the real camera, A / (nu t_z) = W^-1 J / nu for the virtual one.
    const double w00 = Rv(0, 0) - p * Rv(2, 0), w01 = Rv(0, 1) - p * Rv(2, 1);
    const double w10 = Rv(1, 0) - q * Rv(2, 0), w11 = Rv(1, 1) - q * Rv(2, 1);
    const double detW = w00 * w11 - w01 * w10;
    if (!(std::abs(detW) > 1e-12)) return false;
    const double s = 1.0 / (nu * detW);
    const double v00 = s * (w11 * j00 - w01 * j10), v01 = s * (w11 * j01 - w01 * j11);
    const double v10 = s * (-w10 * j00 + w00 * j10), v11 = s * (-w10 * j01 + w00 * j11);

    // Singular values (sig1 >= sig2) and the left singular vector of sig2 of the virtual Jacobian.
    const double a = v00 * v00 + v01 * v01, b = v00 * v10 + v01 * v11, d = v10 * v10 + v11 * v11;
    const double mean = 0.5 * (a + d);
    const double delta = std::hypot(0.5 * (a - d), b);
    const double lambda1 = mean + delta;
    const double lambda2 = std::max(0.0, mean - delta);
    if (!(lambda1 > 0.0) || !std::isfinite(lambda1)) return false;
    const double sigma1 = std::sqrt(lambda1);
    const double gamma = std::sqrt(lambda2 / lambda1);
    const double bMag = std::sqrt(std::max(0.0, 1.0 - gamma * gamma));
    const double axis = 0.5 * std::atan2(2.0 * b, a - d);
    const double u2x = -std::sin(axis), u2y = std::cos(axis);

    // Rows 1 and 2 of the virtual rotation are (A | +-b); row 3 completes the rotation. The sign
    // of b is the two-fold IPPE ambiguity.
    for (int k = 0; k < 2; ++k) {
        const double sign = k == 0 ? 1.0 : -1.0;
        const Vec3d row1{v00 / sigma1, v01 / sigma1, sign * bMag * u2x};
        const Vec3d row2{v10 / sigma1, v11 / sigma1, sign * bMag * u2y};
        const Mat3d Rvirtual = Mat3d::fromRows(row1, row2, cross(row1, row2));
        out[static_cast<std::size_t>(k)].R = Rv * Rvirtual;
        if (!translationForRotation(out[static_cast<std::size_t>(k)].R, rays,
                                    out[static_cast<std::size_t>(k)].t))
            return false;
    }
    return true;
}

Pose3d cvToNeutral(const CvPose& p) {
    return {kNeutralFromCv * p.R * kModelFromTag, kNeutralFromCv * p.t};
}

// ---- multi-tag Levenberg-Marquardt --------------------------------------------------------------

struct PointSet {
    std::vector<Vec3d> world;
    std::vector<sdk::PixelPoint> observed;
};

// Sum of squared pixel residuals of a camera pose (the camera frame in the field frame); infinity
// when a point is behind the camera.
double totalCost(const CameraModel& camera, const Pose3d& fieldToCamera, const PointSet& points) {
    const Pose3d cameraFromField = fieldToCamera.inverse();
    double cost = 0.0;
    for (std::size_t k = 0; k < points.world.size(); ++k) {
        const auto px = camera.project(cameraFromField.apply(points.world[k]));
        if (!px) return kInf;
        const double du = px->x - points.observed[k].x;
        const double dv = px->y - points.observed[k].y;
        cost += du * du + dv * dv;
    }
    return cost;
}

using Mat6 = std::array<std::array<double, 6>, 6>;
using Vec6 = std::array<double, 6>;

bool linearize(const CameraModel& camera, const Pose3d& wc, const PointSet& points, double& cost,
               Mat6& jtj, Vec6& jtr) {
    const Mat3d rt = wc.R.transposed();
    cost = 0.0;
    jtj = {};
    jtr = {};
    for (std::size_t k = 0; k < points.world.size(); ++k) {
        const Vec3d P = rt * (points.world[k] - wc.t);
        if (!(P.x >= CameraModel::kMinDepthM)) return false;
        const double invX = 1.0 / P.x;
        const double u = camera.cxPx() - camera.fxPx() * P.y * invX;
        const double v = camera.cyPx() - camera.fyPx() * P.z * invX;
        const double res[2] = {u - points.observed[k].x, v - points.observed[k].y};
        // d(pixel)/d(P) rows
        const Vec3d jrow[2] = {
            {camera.fxPx() * P.y * invX * invX, -camera.fxPx() * invX, 0.0},
            {camera.fyPx() * P.z * invX * invX, 0.0, -camera.fyPx() * invX},
        };
        for (int i = 0; i < 2; ++i) {
            // Right-multiplied rotation increment: dP/dtheta = [P]x; field-frame translation
            // increment: dP/dt = -R^T.
            const Vec3d gTheta = cross(jrow[i], P);
            const Vec3d gT = -(wc.R * jrow[i]);
            const double g[6] = {gTheta.x, gTheta.y, gTheta.z, gT.x, gT.y, gT.z};
            for (int r = 0; r < 6; ++r) {
                for (int c = 0; c < 6; ++c) jtj[r][c] += g[r] * g[c];
                jtr[r] += g[r] * res[i];
            }
            cost += res[i] * res[i];
        }
    }
    return true;
}

struct LmOutcome {
    Pose3d pose;
    double cost = kInf;
    int iterations = 0;
};

// Levenberg-Marquardt on the pixel residuals, perturbing the pose as R <- R exp([dtheta]x),
// t <- t + dt.
LmOutcome refine(const CameraModel& camera, Pose3d pose, const PointSet& points) {
    constexpr int kMaxIterations = 60;
    constexpr int kMaxDampingTries = 30;
    LmOutcome out;
    out.pose = pose;
    Mat6 jtj;
    Vec6 jtr;
    double cost = 0.0;
    if (!linearize(camera, pose, points, cost, jtj, jtr)) return out;

    double lambda = 1e-3;
    int iteration = 0;
    for (; iteration < kMaxIterations && cost >= 1e-26; ++iteration) {
        bool accepted = false;
        Pose3d candidate;
        double candidateCost = kInf;
        double stepNorm = 0.0;
        for (int attempt = 0; attempt < kMaxDampingTries && !accepted; ++attempt) {
            Mat6 damped = jtj;
            Vec6 rhs{};
            for (int i = 0; i < 6; ++i) {
                damped[i][i] += lambda * std::max(jtj[i][i], 1e-9);
                rhs[i] = -jtr[i];
            }
            Vec6 step{};
            if (solveLinear<6>(damped, rhs, step)) {
                candidate.R = pose.R * rotationFromRotationVector({step[0], step[1], step[2]});
                candidate.t = pose.t + Vec3d{step[3], step[4], step[5]};
                candidateCost = totalCost(camera, candidate, points);
                if (candidateCost < cost) {
                    accepted = true;
                    stepNorm = 0.0;
                    for (const double s : step) stepNorm += s * s;
                    stepNorm = std::sqrt(stepNorm);
                }
            }
            if (!accepted) lambda *= 4.0;
        }
        if (!accepted) break;

        const double previous = cost;
        pose = candidate;
        lambda = std::max(lambda / 3.0, 1e-12);
        if (!linearize(camera, pose, points, cost, jtj, jtr)) break;
        if (stepNorm < 1e-13 || previous - cost <= 1e-15 * previous) {
            ++iteration;
            break;
        }
    }
    out.pose = {orthonormalized(pose.R), pose.t};
    out.cost = cost;
    out.iterations = iteration;
    return out;
}

}  // namespace

double squareReprojectionErrorPx(const CameraModel& camera, const Pose3d& cameraToTag,
                                 double tagSizeM, std::span<const sdk::PixelPoint, 4> cornersPx) {
    const auto projected = projectTagCorners(camera, cameraToTag, tagSizeM);
    if (!projected) return kInf;
    double sum = 0.0;
    for (std::size_t i = 0; i < 4; ++i) {
        const double dx = (*projected)[i].x - cornersPx[i].x;
        const double dy = (*projected)[i].y - cornersPx[i].y;
        sum += dx * dx + dy * dy;
    }
    return std::sqrt(sum / 8.0);
}

namespace {

// Levenberg-Marquardt polish of one candidate on the reprojection error of the four corners (the
// tag frame plays the part of the field frame). The closed-form IPPE rotation loses about half the
// digits when the tag is seen head-on, so an exact fit would not read as exact; the polish never
// increases the error, and the two candidates keep their own minima.
Pose3d polishSquarePose(const CameraModel& camera, double tagSizeM,
                        std::span<const sdk::PixelPoint, 4> cornersPx, const Pose3d& cameraToTag) {
    PointSet points;
    const std::array<Vec3d, 4> local = tagCornersInTagFrame(tagSizeM);
    for (std::size_t i = 0; i < 4; ++i) {
        points.world.push_back(local[i]);
        points.observed.push_back(cornersPx[i]);
    }
    const LmOutcome outcome = refine(camera, cameraToTag.inverse(), points);
    return std::isfinite(outcome.cost) ? outcome.pose.inverse() : cameraToTag;
}

}  // namespace

SquarePnpResult solveSquarePnp(const CameraModel& camera, double tagSizeM,
                               std::span<const sdk::PixelPoint, 4> cornersPx) {
    SquarePnpResult result;
    if (!(tagSizeM > 0.0) || !std::isfinite(tagSizeM)) return result;
    std::array<Vec3d, 4> rays;
    for (std::size_t i = 0; i < 4; ++i) {
        if (!std::isfinite(cornersPx[i].x) || !std::isfinite(cornersPx[i].y)) return result;
        rays[i] = {(cornersPx[i].x - camera.cxPx()) / camera.fxPx(),
                   (cornersPx[i].y - camera.cyPx()) / camera.fyPx(), 1.0};
    }
    std::array<CvPose, 2> cv;
    if (!ippeUnitSquare(rays, cv)) return result;

    std::array<SquarePose, 2> solutions;
    for (std::size_t k = 0; k < 2; ++k) {
        // The model was solved for a unit half-size: the translation scales with the real size.
        const Pose3d closedForm = cvToNeutral({cv[k].R, cv[k].t * (0.5 * tagSizeM)});
        solutions[k].cameraToTag = polishSquarePose(camera, tagSizeM, cornersPx, closedForm);
        solutions[k].reprojErrPx =
            squareReprojectionErrorPx(camera, solutions[k].cameraToTag, tagSizeM, cornersPx);
    }
    if (!std::isfinite(solutions[0].reprojErrPx) && !std::isfinite(solutions[1].reprojErrPx))
        return result;
    const std::size_t best = solutions[1].reprojErrPx < solutions[0].reprojErrPx ? 1 : 0;
    result.best = solutions[best];
    result.alt = solutions[1 - best];
    if (result.best.reprojErrPx >= 1e-9 && std::isfinite(result.alt.reprojErrPx) &&
        result.alt.reprojErrPx > 0.0)
        result.ambiguity = std::clamp(result.best.reprojErrPx / result.alt.reprojErrPx, 0.0, 1.0);
    result.valid = true;
    return result;
}

MultiTagPnpResult solveMultiTagPnp(const CameraModel& camera,
                                   std::span<const TagCornerObservation> tags,
                                   std::span<const SquarePnpResult> seeds) {
    MultiTagPnpResult result;
    if (tags.size() < 2) return result;

    PointSet points;
    points.world.reserve(tags.size() * 4);
    points.observed.reserve(tags.size() * 4);
    for (const TagCornerObservation& tag : tags) {
        const std::array<Vec3d, 4> corners = tagCornersIn(tag.fieldToTag, tag.sizeM);
        for (std::size_t i = 0; i < 4; ++i) {
            points.world.push_back(corners[i]);
            points.observed.push_back(tag.cornersPx[i]);
        }
    }

    std::vector<SquarePnpResult> computed;
    if (seeds.size() != tags.size()) {
        computed.reserve(tags.size());
        for (const TagCornerObservation& tag : tags)
            computed.push_back(solveSquarePnp(camera, tag.sizeM, tag.cornersPx));
        seeds = std::span<const SquarePnpResult>(computed);
    }

    struct Candidate {
        Pose3d fieldToCamera;
        double cost = kInf;
    };
    std::vector<Candidate> candidates;
    candidates.reserve(tags.size() * 2);
    for (std::size_t i = 0; i < tags.size(); ++i) {
        if (!seeds[i].valid) continue;
        for (const SquarePose* pose : {&seeds[i].best, &seeds[i].alt}) {
            Candidate c;
            c.fieldToCamera = tags[i].fieldToTag * pose->cameraToTag.inverse();
            c.cost = totalCost(camera, c.fieldToCamera, points);
            if (std::isfinite(c.cost)) candidates.push_back(c);
        }
    }
    if (candidates.empty()) return result;
    std::stable_sort(candidates.begin(), candidates.end(),
                     [](const Candidate& a, const Candidate& b) { return a.cost < b.cost; });

    // The planar ambiguity can leave the cheapest seed on the wrong side: refine the best few and
    // keep the lowest final cost.
    constexpr std::size_t kStarts = 3;
    LmOutcome best;
    for (std::size_t i = 0; i < std::min(kStarts, candidates.size()); ++i) {
        const LmOutcome outcome = refine(camera, candidates[i].fieldToCamera, points);
        if (outcome.cost < best.cost) best = outcome;
    }
    if (!std::isfinite(best.cost)) return result;
    result.valid = true;
    result.fieldToCamera = best.pose;
    result.reprojErrPx = std::sqrt(best.cost / (2.0 * static_cast<double>(points.world.size())));
    result.iterations = best.iterations;
    return result;
}

}  // namespace wpiemu::core::vision
