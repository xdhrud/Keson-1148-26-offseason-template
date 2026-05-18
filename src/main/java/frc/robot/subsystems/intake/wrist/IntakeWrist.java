package frc.robot.subsystems.intake.wrist;

import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class IntakeWrist extends SubsystemBase {
    private WristIOTalonFX io;
    private WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();
    private final String key = "Intake Wrist";
    private static IntakeWrist instance;
    public SysIdRoutine sysId;
    
    private boolean isStowed = true;
    private boolean stopIntake = false;
    private double positionSetpointRot = 0.0;

    public double getPositionSetpointRot() {
        return positionSetpointRot;
    }

    public static IntakeWrist getInstance() {
        if (instance == null) {
            instance = new IntakeWrist();
        }
        return instance;
    }

    private IntakeWrist() {
        io = new WristIOTalonFX();
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

        isStowed = Math.abs(inputs.wristPositionRot
                - WristConstants.stowedPositionRotations) <= WristConstants.stowToleranceRotations;

        if (isStowed && positionSetpointRot == WristConstants.stowedPositionRotations){
            stopIntake = true;
            runVoltage(0);
        } else {
            stopIntake = false;
        }
        
        Logger.recordOutput(key + "/isStowed", isStowed);
    }

    public boolean isDeployed() {
        return io.isDeployed() && !isStowed;
    }

    public boolean isStowed() {
        return isStowed;
    }

    public double getPositionRot() {
        return inputs.wristPositionRot;
    }

    public void runVoltage(double volts) {
        io.runVoltage(volts);
    }

    public void goToPosition(double rotations) {
        positionSetpointRot = rotations;
        if (!stopIntake){
            Logger.recordOutput(key + "/PositionSetpoint", rotations);
            io.goToPosition(rotations);
        }
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
}
