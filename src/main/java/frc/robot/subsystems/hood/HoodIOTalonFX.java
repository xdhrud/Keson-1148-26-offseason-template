package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class HoodIOTalonFX implements HoodIO {
    private final TalonFX hoodMotor;
    private final PositionVoltage hoodController;

    private final TalonFXConfiguration hoodConfig;

    private final StatusSignal<Angle> hoodPosition;
    private final StatusSignal<AngularVelocity> hoodVelocity;
    private final StatusSignal<Voltage> motorAppliedVoltage;
    private final StatusSignal<Current> motorStatorCurrentAmps;
    private final StatusSignal<Current> motorSupplyCurrentAmps;

    private final Debouncer motorMasterConnectedDebouncer = new Debouncer(0.5);

    public HoodIOTalonFX() {
        hoodMotor = new TalonFX(HoodConstants.motorID, new CANBus("subsystems"));
        hoodController = new PositionVoltage(Units.degreesToRotations(HoodConstants.maxAngleDegrees - 0.1))
                .withEnableFOC(true);
        hoodConfig = new TalonFXConfiguration();
        hoodMotor.setPosition(Units.degreesToRotations(HoodConstants.maxAngleDegrees - 0.1));

        hoodConfig.MotorOutput.Inverted = HoodConstants.motorInverted;
        hoodConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        hoodConfig.Slot0.kP = HoodConstants.kP;
        hoodConfig.Slot0.kI = HoodConstants.kI;
        hoodConfig.Slot0.kD = HoodConstants.kD;
        hoodConfig.Slot0.kS = HoodConstants.kS;
        hoodConfig.Slot0.kV = HoodConstants.kV;
        hoodConfig.Slot0.kA = HoodConstants.kA;

        hoodConfig.MotionMagic.MotionMagicCruiseVelocity = HoodConstants.motionMagicCruiseVelocity;
        hoodConfig.MotionMagic.MotionMagicAcceleration = HoodConstants.motionMagicAcceleration;
        hoodConfig.MotionMagic.MotionMagicJerk = HoodConstants.motionMagicJerk;

        hoodConfig.Feedback.SensorToMechanismRatio = HoodConstants.motorRotationsPerHoodRotation;

        hoodConfig.CurrentLimits.StatorCurrentLimit = HoodConstants.statorLimit;
        hoodConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        // hoodConfig.CurrentLimits.SupplyCurrentLimit = HoodConstants.supplyLimit;
        // hoodConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        hoodConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        hoodConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Units
                .degreesToRotations(HoodConstants.maxAngleDegrees);
        hoodConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        hoodConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Units
                .degreesToRotations(HoodConstants.minAngleDegrees);

        hoodMotor.getConfigurator().apply(hoodConfig);
        hoodMotor.setControl(hoodController);

        hoodPosition = hoodMotor.getPosition();
        hoodVelocity = hoodMotor.getVelocity();
        motorAppliedVoltage = hoodMotor.getMotorVoltage();
        motorStatorCurrentAmps = hoodMotor.getStatorCurrent();
        motorSupplyCurrentAmps = hoodMotor.getSupplyCurrent();
    }

    @Override
    public void updateInputs(HoodIOInputs inputs) {
        inputs.hoodMotorConnected = motorMasterConnectedDebouncer.calculate(hoodMotor.isConnected());
        if (inputs.hoodMotorConnected){
            StatusSignal.refreshAll(hoodPosition, hoodVelocity, motorAppliedVoltage, motorStatorCurrentAmps, motorSupplyCurrentAmps);
        }

        inputs.hoodPositionRot = hoodPosition.getValue().in(Rotations);
        inputs.hoodPositionDegrees = hoodPosition.getValue().in(Degrees);
        inputs.hoodVelocityDPS = hoodVelocity.getValue().in(DegreesPerSecond);
        inputs.hoodAppliedVolts = motorAppliedVoltage.getValue().in(Volts);
        inputs.hoodStatorCurrentAmps = motorStatorCurrentAmps.getValue().in(Amps);
        inputs.hoodSupplyCurrentAmps = motorSupplyCurrentAmps.getValue().in(Amps);
    }

    @Override
    public void runVoltage(double volts) {
        hoodMotor.setControl(new VoltageOut(volts));
    }

    @Override
    public void goToAngle(double degrees) {
        hoodMotor.setControl(hoodController.withPosition(Units.degreesToRotations(degrees)));
    }

    @Override
    public void setTunableConstants(
            double kP,
            double kI,
            double kD,
            double kS,
            double kV,
            double kA) {
        hoodConfig.Slot0.kP = kP;
        hoodConfig.Slot0.kI = kI;
        hoodConfig.Slot0.kD = kD;
        hoodConfig.Slot0.kS = kS;
        hoodConfig.Slot0.kV = kV;
        hoodConfig.Slot0.kA = kA;
        hoodMotor.getConfigurator().apply(hoodConfig, 0.25);
    }
}
