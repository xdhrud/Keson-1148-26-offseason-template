package frc.robot.subsystems.intake.wrist;

import com.ctre.phoenix6.signals.InvertedValue;

public class WristConstants {
    public static final int motorID = 22;
    public static final InvertedValue motorInverted = InvertedValue.Clockwise_Positive;

    public static final double supplyLimit = 40;
    public static final double statorLimit = 80;

    // PID constants
    public static final double kP = 15.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kS = 0.45;
    public static final double kV = 0.0;
    public static final double kG = 0.746;
    public static final double kA = 0.0;

    // Motion magic constants
    public static final double motionMagicCruiseVelocity = 0.125;
    public static final double motionMagicAcceleration = 0.5;
    public static final double motionMagicJerk = 4;

    // Physical constants
    public static final double motorRotationsPerWristRotationRatio = 23.14;

    public static final double stowedPositionRotations = 0.385742;
    public static final double deployedPositionRotations = 0.005;
    public static final double stowToleranceRotations = 0.02;
    public static final double bumpPositionRotations = 0.125;

    public static final double manualVoltage = 4.0;
    public static final double intakeDeployVoltage = -1;
    public static final double shootingVoltage = 3;
}
