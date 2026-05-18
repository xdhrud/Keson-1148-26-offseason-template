package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.roller.Intake;

public class IntakeCommands {

    private IntakeCommands() {}

    public static Command runVoltageCommand(Intake intake, double volts) {
        return intake.run(() -> Intake.runVoltage(volts));
    }
}