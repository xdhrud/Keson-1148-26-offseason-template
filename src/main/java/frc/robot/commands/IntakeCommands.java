package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;

public class IntakeCommands {

    private IntakeCommands() {}

    public static Command runVoltageCommand(Intake intake, double volts) {
        return intake.run(() -> intake.runVoltage(volts));
    }
}