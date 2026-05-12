package frc.robot.subsystems.funnel;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Funnel extends SubsystemBase {
    private FunnelIOTalonFX io;
    private FunnelIOInputsAutoLogged inputs = new FunnelIOInputsAutoLogged();
    private final String key = "Funnel";
    private static Funnel instance;

    public static Funnel getInstance() {
        if (instance == null) {
            instance = new Funnel();
        }
        return instance;
    }

    private Funnel() {
        io = new FunnelIOTalonFX();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs(key, inputs);
    }

    public void runVoltageFunnel(double volts) {
        Logger.recordOutput(key + "/FunnelVoltage", volts);
        io.runVoltageFunnel(volts);
    }

    public void runVoltageIndexer(double volts) {
        io.runVoltageIndexer(volts);
    }
}
