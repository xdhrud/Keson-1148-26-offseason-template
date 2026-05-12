package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    private ShooterIOTalonFX io;
    private ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
    private final String key = "Shooter";
    private static Shooter instance;

    @AutoLogOutput
    private double velocitySetpointRPS;

    public double getVelocitySetpointRPS() {
        return velocitySetpointRPS;
    }

    public static Shooter getInstance() {
        if (instance == null) {
            instance = new Shooter();
        }
        return instance;
    }

    private Shooter() {
        io = new ShooterIOTalonFX();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs(key, inputs);
    }

    public void runVoltage(double volts) {
        io.runVoltage(volts);
    }

    public void runVelocity(double velocityRPS) {
        velocitySetpointRPS = velocityRPS;
        io.runVelocity(velocityRPS);
    }

    public boolean isWithinVelocityThreshold() {
        return Math.abs(inputs.shooterVelocityRPS - velocitySetpointRPS) <= ShooterConstants.velocityThresholdRPS;
    }
}
