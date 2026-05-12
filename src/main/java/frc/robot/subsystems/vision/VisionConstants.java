package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.measure.Distance;

public class VisionConstants {
    // Limelight Standard Deviation Coefficients
    public static final double xyStdDev = 3.0;
    public static final double rStdDev = 0.1;

    public static final double maxAmbiguity = 0.27;
    public static final Distance maxDistToCamera = Meters.of(6.5);

    public static final String[] limelightNames = {
            "limelight-back", "limelight-left", "limelight-right"
    };
}
