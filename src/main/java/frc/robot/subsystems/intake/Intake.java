package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.IntakeConstants.IntakeState;

public class Intake extends SubsystemBase {
    private IntakeIOTalonFX io;
    private IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
    private final String key = "Intake";
    private static Intake instance;

    @AutoLogOutput
    private IntakeState state = IntakeState.OFF;

    public IntakeState getState() {
        return state;
    }

    public Command setState(IntakeState state) {
        return Commands.runOnce(() -> this.state = state);
    }

    public static Intake getInstance() {
        if (instance == null) {
            return new Intake();
        }
        return instance;
    }

    public Intake() {
        io = new IntakeIOTalonFX();
    }

    public void runVoltage(double volts) {
        Logger.recordOutput(key + "/TargetVoltage", volts);
        io.runVoltage(volts);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs(key, inputs);

        switch (state) {
            case OFF:
                runVoltage(0);
                break;
            case FAST:
                runVoltage(IntakeConstants.fastIntakeVoltage);
                break;
            case SLOW:
                runVoltage(IntakeConstants.slowIntakeVoltage);
                break;
            case REVERSE:
                runVoltage(IntakeConstants.reverseIntakeVoltage);
                break;
        }
    }
}
