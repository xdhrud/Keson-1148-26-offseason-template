package frc.robot.subsystems.intake.roller;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Roller extends SubsystemBase {
    private RollerIOTalonFX io;
    private RollerIOInputsAutoLogged inputs = new RollerIOInputsAutoLogged();
    private final String key = "Roller";
    private static Roller instance;

    public static Roller getInstance() {
        if (instance == null) {
            instance = new Roller();
        }
        return instance;
    }

    public Roller() {
        io = new RollerIOTalonFX();
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
