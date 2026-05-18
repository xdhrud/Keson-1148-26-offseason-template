package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.roller.Roller;
import frc.robot.subsystems.intake.roller.RollerConstants;
import frc.robot.subsystems.intake.wrist.IntakeWrist;
import frc.robot.subsystems.intake.wrist.WristConstants;

public class Intake extends SubsystemBase {
    private static Intake instance;

    public Intake() {}

    public static Intake getInstance() {
        if (instance == null) {
            instance = new Intake();
        }
        return instance;
    }

    public static enum IntakeState {
        OFF,
        FAST,
        SLOW,
        REVERSE
    }

    @AutoLogOutput
    private IntakeState state = IntakeState.OFF;
    private IntakeState previousState = state;

    public IntakeState getState() {
        return state;
    }

    public Command setState(IntakeState state) {
        return Commands.runOnce(() -> this.state = state);
    }

    public void setStateVoid(IntakeState state) {
        this.state = state;
    }

    public IntakeState getPreviousState() {
        return previousState;
    }

    @Override
    public void periodic() {
        // State based
        switch (state) {
        case OFF:
            Roller.getInstance().runVoltage(0);
            IntakeWrist.getInstance().goToPosition(WristConstants.stowedPositionRotations);
            break;
        case FAST:
            Roller.getInstance().runVoltage(RollerConstants.fastRollerVoltage);
            IntakeWrist.getInstance().goToPosition(WristConstants.deployedPositionRotations);
            break;
        case SLOW:
            Roller.getInstance().runVoltage(RollerConstants.slowRollerVoltage);
            IntakeWrist.getInstance().goToPosition(WristConstants.deployedPositionRotations);
            break;
        case REVERSE:
            Roller.getInstance().runVoltage(RollerConstants.reverseRollerVoltage);
            IntakeWrist.getInstance().goToPosition(WristConstants.deployedPositionRotations);
            break;
        }

        // Set previous state for operator state switches
        if (state != IntakeState.REVERSE) {
            previousState = state;
        }
    }
}
