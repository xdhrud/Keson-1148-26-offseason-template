package frc.robot.subsystems.intake.roller;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class RollerIOTalonFX implements RollerIO {
    // Motors and intake controllers
    private final TalonFX intakeMotorMaster;
    private final TalonFX intakeMotorFollower;
    private final VoltageOut intakeMasterController;
    private final Follower intakeFollowerController;

    private final TalonFXConfiguration intakeConfig;

    private final StatusSignal<AngularVelocity> intakeVelocity;
    private final StatusSignal<Voltage> motorAppliedVoltage;
    private final StatusSignal<Current> motorStatorCurrentAmps;
    private final StatusSignal<Current> motorSupplyCurrentAmps;
    private final StatusSignal<Temperature> motorTempC;

    public final Debouncer motorMasterConnectedDebouncer = new Debouncer(0.5);
    public final Debouncer motorFollowerConnectedDebouncer = new Debouncer(0.5);

    public RollerIOTalonFX() {
        intakeMotorMaster = new TalonFX(RollerConstants.motorMasterID, new CANBus("subsystems"));
        intakeMotorFollower = new TalonFX(RollerConstants.motorFollowerID, new CANBus("subsystems"));
        intakeMasterController = new VoltageOut(0.0).withEnableFOC(true);
        intakeFollowerController = new Follower(intakeMotorMaster.getDeviceID(),
                RollerConstants.motorFollowerAligned);

        intakeConfig = new TalonFXConfiguration();

        intakeConfig.MotorOutput.Inverted = RollerConstants.motorMasterInverted;
        intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        intakeConfig.CurrentLimits.StatorCurrentLimit = RollerConstants.statorLimit;
        intakeConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        // intakeConfig.CurrentLimits.SupplyCurrentLimit = IntakeConstants.supplyLimit;
        // intakeConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        intakeMotorMaster.getConfigurator().apply(intakeConfig);
        intakeMotorFollower.getConfigurator().apply(intakeConfig);
        intakeMotorMaster.setControl(intakeMasterController);
        intakeMotorFollower.setControl(intakeFollowerController);

        intakeVelocity = intakeMotorMaster.getVelocity();
        motorAppliedVoltage = intakeMotorMaster.getMotorVoltage();
        motorStatorCurrentAmps = intakeMotorMaster.getStatorCurrent();
        motorSupplyCurrentAmps = intakeMotorMaster.getSupplyCurrent();
        motorTempC = intakeMotorMaster.getDeviceTemp();
    }

    @Override
    public void updateInputs(RollerIOInputs inputs) {
        inputs.intakeMotorMasterConnected = motorMasterConnectedDebouncer.calculate(intakeMotorMaster.isConnected());
        inputs.intakeMotorFollowerConnected = motorFollowerConnectedDebouncer
                .calculate(intakeMotorFollower.isConnected());
        
        if (inputs.intakeMotorMasterConnected && inputs.intakeMotorFollowerConnected){
            StatusSignal.refreshAll(intakeVelocity, motorAppliedVoltage, motorStatorCurrentAmps, motorSupplyCurrentAmps, motorTempC);
        }

        inputs.intakeVelocityRPS = intakeVelocity.getValue().in(RotationsPerSecond);
        inputs.intakeAppliedVolts = motorAppliedVoltage.getValue().in(Volts);
        inputs.intakeStatorCurrentAmps = motorStatorCurrentAmps.getValue().in(Amps);
        inputs.intakeSupplyCurrentAmps = motorSupplyCurrentAmps.getValue().in(Amps);
        inputs.intakeTempC = motorTempC.getValue().in(Celsius);
    }

    public void runVoltage(double volts) {
        intakeMotorMaster.setControl(new VoltageOut(volts));
    }
}
