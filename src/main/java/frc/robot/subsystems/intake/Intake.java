package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private IntakeIOTalonFX io;
    private IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
    private final String key = "Intake";
    private static Intake instance;

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
    }
}
