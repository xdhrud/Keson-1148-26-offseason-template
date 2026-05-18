package frc.robot.subsystems.intake.roller;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.math.util.Units;

public class RollerConstants {

    // Motor constants
    public static final int motorMasterID = 20;
    public static final int motorFollowerID = 21;
    public static final InvertedValue motorMasterInverted = InvertedValue.CounterClockwise_Positive;
    public static final MotorAlignmentValue motorFollowerAligned = MotorAlignmentValue.Opposed;
    public static final double statorLimit = 40;

    // Physical constants
    public static final double motorRotationsPerIntakeRotationRatio = 2;
    public static final double intakeWheelRadiusMeters = Units.inchesToMeters(1.5);

    public static final double fastRollerVoltage = 11.0;
    public static final double slowRollerVoltage = 0.5;
    public static final double reverseRollerVoltage = -4.0;
}
