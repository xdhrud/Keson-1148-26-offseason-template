package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class Hood extends SubsystemBase {
    private HoodIOTalonFX io;
    private HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();
    private final String key = "Hood";
    private static Hood instance = null;
    public SysIdRoutine sysId;

    @AutoLogOutput
    private double hoodAngleSetpointDeg = HoodConstants.maxAngleDegrees - 0.1;

    public static Hood getInstance() {
        if (instance == null) {
            instance = new Hood();
        }
        return instance;
    }

    private Hood() {
        io = new HoodIOTalonFX();
        sysId = new SysIdRoutine(
                new SysIdRoutine.Config(
                        null,
                        null,
                        null,
                        (state) -> Logger.recordOutput(key + "/SysIdState", state.toString())),
                new SysIdRoutine.Mechanism((voltage) -> runVoltage(voltage.in(Volts)), null, this));
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs(key, inputs);
    }

    public void runVoltage(double volts) {
        io.runVoltage(volts);
    }

    public void goToAngle(double degrees) {
        hoodAngleSetpointDeg = degrees;
        io.goToAngle(degrees);
    }

    /** Returns a command to run a quasistatic test in the specified direction. */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return run(() -> runVoltage(0.0))
                .withTimeout(1.0)
                .andThen(sysId.quasistatic(direction));
    }

    /** Returns a command to run a dynamic test in the specified direction. */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return run(() -> runVoltage(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
    }

    public boolean isWithinAngleThreshold() {
        return Math.abs(inputs.hoodPositionDegrees - hoodAngleSetpointDeg) <= HoodConstants.angleThresholdDeg;
    }
}