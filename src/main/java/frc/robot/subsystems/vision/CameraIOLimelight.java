package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LimelightHelpers.PoseEstimate;

public class CameraIOLimelight implements CameraIO {
    private String name = "";

    private boolean updateElasticPose = false;

    public String getName() {
        return name;
    }

    private boolean useMegaTag2 = false;

    public void setUseMegaTag2(boolean useMegaTag2) {
        this.useMegaTag2 = useMegaTag2;
    }

    private PoseEstimate currentPose;

    public CameraIOLimelight(String name) {
        this.name = name;
        currentPose = new PoseEstimate();
    }

    @Override
    public void updateInputs(CameraIOInputs inputs) {
        if (useMegaTag2) {
            currentPose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
        } else {
            currentPose = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);
        }
        if (currentPose == null) {
            return;
        }
        inputs.tagCount = currentPose.tagCount;
        if (currentPose.tagCount != 0) {
            inputs.ambiguity = currentPose.rawFiducials[0].ambiguity;
            inputs.distToCamera = Meters.of(currentPose.rawFiducials[0].distToCamera);
        }
        inputs.estimatedPose = currentPose.pose;
        if (updateElasticPose) {
            Drive.getInstance().field.getObject(name + " pose").setPose(inputs.estimatedPose);
            updateElasticPose = false;
        }
    }

    @Override
    public void updateElasticPose() {
        updateElasticPose = true;
    }

    public void setThrottle(int throttle) {
        LimelightHelpers.SetThrottle(name, throttle);
    }

    public void setRobotOrientation(Rotation2d yaw) {
        LimelightHelpers.SetRobotOrientation(name, yaw.getDegrees(), 0, 0, 0, 0, 0);
    }

    public void setIMUMode(int mode) {
        LimelightHelpers.SetIMUMode(name, mode);
    }

    public void recordMatch(boolean isAuto) {
        if (isAuto)
            LimelightHelpers.triggerRewindCapture(name, 20);
        else
            LimelightHelpers.triggerRewindCapture(name, 140);
    }

    @Override
    public TimestampedPose getTimestampedPose() {
        if (!shouldAcceptPose(currentPose)) {
            return null;
        }

        return new TimestampedPose(currentPose.pose, currentPose.timestampSeconds,
                Math.max(Math.pow(currentPose.avgTagDist, 1.8), 0.5) / currentPose.tagCount
                        * Math.sqrt(currentPose.rawFiducials[0].ambiguity));
    }

    private boolean shouldAcceptPose(PoseEstimate pose) {
        if (pose == null) {
            return false;
        }
        if (pose.tagCount == 0) {
            return false;
        }

        if (pose.rawFiducials[0].ambiguity > VisionConstants.maxAmbiguity) {
            return false;
        }

        if (pose.rawFiducials[0].distToCamera > VisionConstants.maxDistToCamera.in(Meters)) {
            return false;
        }
        return true;
    }
}
