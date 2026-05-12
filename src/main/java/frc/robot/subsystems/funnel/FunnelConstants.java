package frc.robot.subsystems.funnel;

import com.ctre.phoenix6.signals.InvertedValue;

public class FunnelConstants {
    // Motor constants
    public static final int funnelMotorID = 13;
    public static final int indexerMotorID = 14;
    public static final InvertedValue funnelInverted = InvertedValue.Clockwise_Positive;
    public static final InvertedValue indexerInverted = InvertedValue.Clockwise_Positive;
    public static final double funnelStatorLimit = 80;
    public static final double funnelSupplyLimit = 40;
    public static final double indexerStatorLimit = 80;
    public static final double indexerSupplyLimit = 40;

    // Runtime constants
    public static final double funnelSlowVolts = 0;
    public static final double funnelFastVolts = 12;
    public static final double indexerSlowVolts = -1;
    public static final double indexerFastVolts = 10;
    public static final double passingFunnelVolts = 12;
    public static final double passingIndexerVolts = 12;
}
