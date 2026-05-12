package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Meters;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Distance;

public interface CameraIO {
    // @AutoLog
    public static class TimestampedPose {
        public Pose2d pose;
        public double timestamp;
        public double stdMultiplier;

        public TimestampedPose(Pose2d pose, double timestamp, double stdMultiplier) {
            this.pose = pose;
            this.timestamp = timestamp;
            this.stdMultiplier = stdMultiplier;
        }

        public TimestampedPose() {
        }
    }

    @AutoLog
    public static class CameraIOInputs {
        public int tagCount = 0;
        public double ambiguity = 0.0;
        public Distance distToCamera = Meters.of(0.0);
        public Pose2d estimatedPose;
    }

    public default void updateInputs(CameraIOInputs inputs) {
    }

    public default void updateElasticPose() {
    }

    public default TimestampedPose getTimestampedPose() {
        return null;
    }
}
