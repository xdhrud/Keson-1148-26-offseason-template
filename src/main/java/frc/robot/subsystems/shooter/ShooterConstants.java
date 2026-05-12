package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class ShooterConstants {
    // Motor constants
    public static final int motorMasterID = 15;
    public static final int motorFollower1ID = 16;
    public static final int motorFollower2ID = 17;
    public static final int motorFollower3ID = 18;
    public static final InvertedValue motorMasterInverted = InvertedValue.CounterClockwise_Positive;
    public static final MotorAlignmentValue motorFollower1Aligned = MotorAlignmentValue.Aligned;
    public static final MotorAlignmentValue motorFollower2Aligned = MotorAlignmentValue.Opposed;
    public static final MotorAlignmentValue motorFollower3Aligned = MotorAlignmentValue.Opposed;
    // Higher supply limit with a time limit to allow for 50A during spoolup
    public static final double supplyLimit = 50;
    // Time it takes to spool up
    public static final double supplyLimitTime = 1.0;
    // Nominal supply limit
    public static final double supplyLimitLower = 40;
    public static final double statorLimit = 60;

    // PID constants
    public static final double kP = 0.35;
    public static final double kI = 0.1;
    public static final double kD = 0;
    public static final double kS = 0.3 / 4.0;
    public static final double kV = 0.39 / 4.0;
    public static final double kA = 0.0;

    public static final double motionMagicAcceleration = 50;
    public static final double motionMagicJerk = 500;

    // Physical constants
    public static final double motorRotationsPerShooterRotationRatio = 3.0 / 4.0;

    // Velocity Constants
    public static final double nearRPS = 46.0;
    public static final double middleRPS = 48.0;
    public static final double towerRPS = 48.0;
    public static final double trenchRPS = 50.0;
    public static final double defaultRPS = 40.0;

    public static final double passingNeutralRPS = 59.0;
    public static final double passingOpponentRPS = 120.0;

    public static final double velocityThresholdRPS = 5.0;
    public static final double minVelocityRPS = 0.0;
    public static final double maxVelocityRPS = 128.0;

    public static final InterpolatingDoubleTreeMap hoodAngleMap = new InterpolatingDoubleTreeMap();
    public static final InterpolatingDoubleTreeMap shooterVelocityMap = new InterpolatingDoubleTreeMap();
    public static final InterpolatingDoubleTreeMap timeOfFlightMap = new InterpolatingDoubleTreeMap();
    public static final InterpolatingDoubleTreeMap indexerVoltageMap = new InterpolatingDoubleTreeMap();
    // Distance between estimated time and more accurate time (next iteration) at which it will stop optimization
    // Don't make this too low, or it will cause loop overruns, and don't make it too high, or else it will miss
    public static final double sotmOptimizationTimeThresholdSeconds = 0.5;
    public static final double sotmPhaseShiftTimeSeconds = 0.02;
}