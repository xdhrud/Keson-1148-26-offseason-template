package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.intake.roller.Roller;
import frc.robot.subsystems.intake.roller.RollerConstants;

public class Intake {

    public Intake() {}

    public static enum IntakeState {
        OFF,
        FAST,
        SLOW,
        REVERSE
    }

    @AutoLogOutput
    private IntakeState state = IntakeState.OFF;

    public IntakeState getState() {
        return state;
    }

    public Command setState(IntakeState state) {
        return Commands.runOnce(() -> this.state = state);

        switch (state) {
        case OFF:
            Roller.getInstance().runVoltage(0);
            break;
        case FAST:
            Roller.getInstance().runVoltage(RollerConstants.fastRollerVoltage);
            break;
        case SLOW:
            Roller.getInstance().runVoltage(RollerConstants.slowRollerVoltage);
            break;
        case REVERSE:
            Roller.getInstance().runVoltage(RollerConstants.reverseRollerVoltage);
            break;
        }
    }
}
