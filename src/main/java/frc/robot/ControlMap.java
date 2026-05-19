package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer.Mode;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakeState;

public class ControlMap {
    private static ControlMap instance;

    public static ControlMap getInstance() {
        if (instance == null) {
            instance = new ControlMap();
        }
        return instance;
    }

    private ControlMap() {
    }

    public void configurePreset1(CommandXboxController operator, CommandPS5Controller driver) {
        // -------- DRIVER CONTROLS ---------

        // Reset gyro to 0° when circle button is pressed
        driver.circle().onTrue(Commands.runOnce(RobotContainer.currentMode == Mode.SIM ? () ->
        // simulation
        Drive.getInstance().setPose(RobotContainer.driveSimulation.getSimulatedDriveTrainPose())
                : () ->
                // real/test
                Drive.getInstance().setPose(new Pose2d(
                        Drive.getInstance().getPose().getTranslation(),
                        DriverStation.getAlliance().get() == Alliance.Blue ? new Rotation2d()
                                : new Rotation2d(Math.PI))),
                Drive.getInstance()).ignoringDisable(true));

        // Intake toggle L2
        driver.L2().onTrue(Commands.either(
                Intake.getInstance().setState(IntakeState.SLOW),
                Intake.getInstance().setState(IntakeState.FAST),
                () -> Intake.getInstance().getState() == IntakeState.FAST
        ));

        // -------- OPERATOR CONTROLS ---------

        operator.leftBumper().onTrue(Intake.getInstance().setState(IntakeState.REVERSE))
                .onFalse(Commands.runOnce(() ->
                Intake.getInstance().setStateVoid(Intake.getInstance().getPreviousState())));

        // TODO: Add controls here
    }
}
