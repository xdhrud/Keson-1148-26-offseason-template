package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.CameraIO.TimestampedPose;

public class Vision extends SubsystemBase {
    private static Vision instance;
    private CameraIOLimelight[] cameras;
    private CameraIOInputsAutoLogged[] inputs;
    private String key = "Vision";

    public static Vision getInstance() {
        if (instance == null) {
            instance = new Vision();
        }
        return instance;
    }

    private Vision() {
        cameras = new CameraIOLimelight[VisionConstants.limelightNames.length];
        inputs = new CameraIOInputsAutoLogged[cameras.length];
        for (int i = 0; i < cameras.length; i++) {
            cameras[i] = new CameraIOLimelight(VisionConstants.limelightNames[i]);
            inputs[i] = new CameraIOInputsAutoLogged();
        }
    }

    @Override
    public void periodic() {
        boolean seeingTags = true;
        for (int i = 0; i < inputs.length; i++) {
            cameras[i].updateInputs(inputs[i]);
            if (inputs[i].tagCount == 0) {
                seeingTags = false;
            }
            Logger.processInputs(key + "/" + cameras[i].getName(), inputs[i]);
        }

        SmartDashboard.putBoolean("SeeingTags", seeingTags);
    }

    public void updateElasticPose() {
        for (CameraIOLimelight camera : cameras) {
            camera.updateElasticPose();
        }
    }

    public void setUseMegaTag2(boolean useMegaTag2) {
        for (CameraIOLimelight camera : cameras) {
            camera.setUseMegaTag2(useMegaTag2);
        }
    }

    public void setThrottle(int throttle) {
        for (CameraIOLimelight camera : cameras) {
            camera.setThrottle(throttle);
        }
    }

    public void setRobotOrientation(Rotation2d yaw) {
        for (CameraIOLimelight camera : cameras) {
            camera.setRobotOrientation(yaw);
        }
    }

    public void setIMUMode(int mode) {
        for (CameraIOLimelight camera : cameras) {
            camera.setIMUMode(mode);
        }
    }

     public void recordMatch(boolean isAuto) {
        for (CameraIOLimelight camera : cameras) {
            camera.recordMatch(isAuto);
        }
    }

    // @AutoLogOutput(key = "Vision/TimestampedPoses")
    public TimestampedPose[] getTimestampedPoses() {
        TimestampedPose[] poses = new TimestampedPose[cameras.length];
        for (int i = 0; i < cameras.length; i++) {
            poses[i] = cameras[i].getTimestampedPose();
        }
        return poses;
    }
}
