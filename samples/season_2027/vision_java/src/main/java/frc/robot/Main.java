package frc.robot;

import java.util.Locale;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.wpilib.framework.RobotBase;
import org.wpilib.framework.TimedRobot;
import org.wpilib.networktables.NetworkTable;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.system.Timer;

public final class Main {
    private Main() {}

    public static void main(String... args) {
        RobotBase.startRobot(Robot::new);
    }

    public static final class Robot extends TimedRobot {
        private final PhotonCamera photon = new PhotonCamera("front");
        private final NetworkTable limelight =
                NetworkTableInstance.getDefault().getTable("limelight-front");
        private PhotonPipelineResult result = new PhotonPipelineResult();
        private double lastPrint;
        private double startedAt = Double.NaN;

        @Override
        public void robotPeriodic() {
            double now = Timer.getMonotonicTimestamp();
            if (Double.isNaN(startedAt)) startedAt = now;
            double elapsed = now - startedAt;
            photon.setDriverMode(elapsed >= 1.0 && elapsed < 1.5);
            photon.setEnabled(!(elapsed >= 2.0 && elapsed < 2.5));
            photon.setPipelineIndex(elapsed >= 2.5 ? 2 : 0);
            for (var next : photon.getAllUnreadResults()) result = next;
            if (now - lastPrint < 0.2) return;
            lastPrint = now;
            double[] pose = limelight.getEntry("botpose_wpiblue").getDoubleArray(new double[0]);
            var multi = result.getMultiTagResult();
            double x = multi.map(value -> value.estimatedPose.best.getX()).orElse(0.0);
            double y = multi.map(value -> value.estimatedPose.best.getY()).orElse(0.0);
            var target = result.hasTargets() ? result.getBestTarget() : null;
            System.out.printf(Locale.ROOT,
                    "VISION2027 ll=%d llTags=%.0f llX=%.4f llY=%.4f llLatency=%.1f llAge=%.2f"
                    + " photon=%d tags=%d multi=%d x=%.4f y=%.4f age=%.2f sequence=%d pipeline=%d driver=%d enabled=%d"
                    + " latency=%.2f id=%d targetX=%.4f corners=%d calibration=%d%n",
                    limelight.getEntry("tv").getDouble(0) == 1 ? 1 : 0,
                    pose.length > 7 ? pose[7] : 0,
                    pose.length > 0 ? pose[0] : 0, pose.length > 1 ? pose[1] : 0,
                    limelight.getEntry("tl").getDouble(0) + limelight.getEntry("cl").getDouble(0),
                    (now - limelight.getEntry("hb").getLastChange() / 1e9) * 1000,
                    photon.isConnected() ? 1 : 0, result.getTargets().size(), multi.isPresent() ? 1 : 0,
                    x, y, (now - result.getTimestampSeconds()) * 1000, result.metadata.sequenceID,
                    photon.getPipelineIndex(), photon.getDriverMode() ? 1 : 0, photon.getEnabled() ? 1 : 0,
                    (result.metadata.publishTimestampNanos - result.metadata.captureTimestampNanos) / 1e6,
                    target == null ? -1 : target.getFiducialId(),
                    target == null ? 0 : target.getBestCameraToTarget().getX(),
                    target == null ? 0 : target.getDetectedCorners().size(),
                    photon.getCameraMatrix().isPresent() && photon.getDistCoeffs().isPresent() ? 1 : 0);
            System.out.flush();
        }
    }
}
