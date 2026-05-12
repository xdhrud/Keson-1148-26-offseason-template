package frc.robot.subsystems.hood;

import com.ctre.phoenix6.signals.InvertedValue;

public class HoodConstants {
    public static final int motorID = 19;
    public static final InvertedValue motorInverted = InvertedValue.CounterClockwise_Positive;
    public static final double statorLimit = 40;

    // PID constants
    public static final double kP = 75.0;
    public static final double kI = 1;
    public static final double kD = 0.0;
    public static final double kS = 0.45;
    public static final double kV = 0.124;
    public static final double kA = 0.0;

    // Motion magic constants
    public static final double motionMagicCruiseVelocity = 0.342;
    public static final double motionMagicAcceleration = 6;
    public static final double motionMagicJerk = 80;

    // Max and min angles (degrees)
    public static final double maxAngleDegrees = 87.2;
    public static final double minAngleDegrees = 56.42;
    public static final double angleThresholdDeg = 2;

    // Physical constants
    public static final double motorRotationsPerHoodRotation = 96.7;

    // Positions
    public static final double hoodAnglePassNeutralZone = 56.42;
    public static final double hoodAnglePassOpponentZone = 56.42;
    public static final double hoodAngleShootTrench = 59;
    public static final double hoodAngleShootMiddle = 59.0; 
    public static final double hoodAngleShootTower = 67.0;
    public static final double hoodAngleShootNear = 75.0;
}