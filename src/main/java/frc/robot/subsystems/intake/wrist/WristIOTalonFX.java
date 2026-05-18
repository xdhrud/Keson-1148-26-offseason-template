package frc.robot.subsystems.intake.wrist;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class WristIOTalonFX implements WristIO {
    private final TalonFX wristMotor;
    private final PositionVoltage wristController;

    private final TalonFXConfiguration wristConfig;

    private final StatusSignal<Angle> wristPosition;
    private final StatusSignal<AngularVelocity> wristVelocity;
    private final StatusSignal<Voltage> motorAppliedVoltage;
    private final StatusSignal<Current> motorStatorCurrentAmps;
    private final StatusSignal<Current> motorSupplyCurrentAmps;

    private final Debouncer motorConnectedDebouncer = new Debouncer(0.5);

    public WristIOTalonFX() {
        wristMotor = new TalonFX(WristConstants.motorID, new CANBus("subsystems"));
        wristController = new PositionVoltage(WristConstants.stowedPositionRotations).withEnableFOC(true);
        wristConfig = new TalonFXConfiguration();

        wristMotor.setPosition(WristConstants.stowedPositionRotations);

        wristConfig.MotorOutput.Inverted = WristConstants.motorInverted;
        wristConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        wristConfig.CurrentLimits.StatorCurrentLimit = WristConstants.statorLimit;
        wristConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        wristConfig.CurrentLimits.SupplyCurrentLimit = WristConstants.supplyLimit;
        wristConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        wristConfig.Slot0.kP = WristConstants.kP;
        wristConfig.Slot0.kI = WristConstants.kI;
        wristConfig.Slot0.kD = WristConstants.kD;
        wristConfig.Slot0.kS = WristConstants.kS;
        wristConfig.Slot0.kV = WristConstants.kV;
        wristConfig.Slot0.kG = WristConstants.kG;
        wristConfig.Slot0.kA = WristConstants.kA;
        wristConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

        wristConfig.MotionMagic.MotionMagicCruiseVelocity = WristConstants.motionMagicCruiseVelocity;
        wristConfig.MotionMagic.MotionMagicAcceleration = WristConstants.motionMagicAcceleration;
        wristConfig.MotionMagic.MotionMagicJerk = WristConstants.motionMagicJerk;

        wristConfig.Feedback.SensorToMechanismRatio = WristConstants.motorRotationsPerWristRotationRatio;

        wristConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        wristConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = WristConstants.stowedPositionRotations;
        wristConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        wristConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = WristConstants.deployedPositionRotations;

        wristMotor.getConfigurator().apply(wristConfig);
        wristMotor.setControl(wristController);

        wristPosition = wristMotor.getPosition();
        wristVelocity = wristMotor.getVelocity();
        motorAppliedVoltage = wristMotor.getMotorVoltage();
        motorStatorCurrentAmps = wristMotor.getStatorCurrent();
        motorSupplyCurrentAmps = wristMotor.getSupplyCurrent();
    }

    @Override
    public void updateInputs(WristIOInputs inputs) {
        inputs.wristMotorConnected = motorConnectedDebouncer.calculate(wristMotor.isConnected());
        if ((inputs.wristMotorConnected)){
            StatusSignal.refreshAll(wristPosition, wristVelocity, motorAppliedVoltage, motorStatorCurrentAmps, motorSupplyCurrentAmps);
        }

        inputs.wristPositionRot = wristPosition.getValue().in(Rotations);
        inputs.wristVelocityRPS = wristVelocity.getValue().in(RotationsPerSecond);
        inputs.wristAppliedVolts = motorAppliedVoltage.getValue().in(Volts);
        inputs.wristStatorCurrentAmps = motorStatorCurrentAmps.getValue().in(Amps);
        inputs.wristSupplyCurrentAmps = motorSupplyCurrentAmps.getValue().in(Amps);
    }

    @Override
    public void runVoltage(double volts) {
        wristMotor.setControl(new VoltageOut(volts));
    }

    public boolean isDeployed() {
        return Math.abs(wristVelocity.getValue().in(RotationsPerSecond)) <= 0.2;
    }

    @Override
    public void goToPosition(double rotations) {
        wristMotor.setControl(wristController.withPosition(rotations));
    }
}
