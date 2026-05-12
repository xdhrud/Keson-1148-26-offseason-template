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
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.wrist.IntakeWrist;
import frc.robot.subsystems.wrist.WristConstants;
import frc.robot.subsystems.intake.Intake;

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

        // Intake Controls
        driver.L1().onTrue(Commands.either(
                Commands.runOnce(() -> {
                    IntakeWrist.getInstance().goToPosition(WristConstants.deployedPositionRotations);
                    Intake.getInstance().runVoltage(IntakeConstants.fastIntakeVoltage);
                }, IntakeWrist.getInstance(), Intake.getInstance())
                        .andThen(Commands.waitUntil(() -> IntakeWrist.getInstance().isDeployed()))
                        .andThen(Commands.runOnce(
                                () -> IntakeWrist.getInstance().runVoltage(WristConstants.intakeDeployVoltage),
                                IntakeWrist.getInstance())),
                Commands.either(
                        Commands.run(() -> {
                            IntakeWrist.getInstance().runVoltage(WristConstants.shootingVoltage);
                            Intake.getInstance().runVoltage(IntakeConstants.fastIntakeVoltage);
                        }, IntakeWrist.getInstance(), Intake.getInstance())
                                .until(() -> IntakeWrist.getInstance().isStowed()).andThen(Commands.runOnce(() -> {
                                    Intake.getInstance().runVoltage(0);
                                    IntakeWrist.getInstance().goToPosition(WristConstants.stowedPositionRotations);
                                }, IntakeWrist.getInstance(), Intake.getInstance())),
                        Commands.runOnce(() -> {
                            Intake.getInstance().runVoltage(0);
                            IntakeWrist.getInstance().goToPosition(WristConstants.stowedPositionRotations);
                        }, IntakeWrist.getInstance(), Intake.getInstance()),
                        () -> RobotContainer.shooterCommand.isRunningFunnel()),
                () -> IntakeWrist.getInstance().getPositionSetpointRot() == WristConstants.stowedPositionRotations));

        driver.L2().onTrue(Commands.runOnce(() -> {
            Intake.getInstance().runVoltage(IntakeConstants.reverseIntakeVoltage);
        }, Intake.getInstance())).onFalse(Commands.runOnce(() -> {
            if (IntakeWrist.getInstance().getPositionSetpointRot() == WristConstants.stowedPositionRotations) {
                Intake.getInstance().runVoltage(0);
            } else {
                Intake.getInstance().runVoltage(IntakeConstants.fastIntakeVoltage);
            }
        }, IntakeWrist.getInstance(), Intake.getInstance()));

        // Funnel Controls
        driver.R1().onTrue(Commands.runOnce(() -> {
            RobotContainer.shooterCommand.runFunnel();
        })).onFalse(Commands.runOnce(() -> {
            RobotContainer.shooterCommand.stopFunnel();
        }));

        // Shooting Controls
        driver.R2().onTrue(Commands.runOnce(() -> {
            RobotContainer.shooterCommand.setAutoRotating(true);
        })).onFalse(Commands.runOnce(() -> {
            RobotContainer.shooterCommand.setAutoRotating(false);
        }));

        // --------------- OPERATOR CONTROLS ---------------

        operator.povDown().onTrue(Commands.runOnce(() -> {
            Drive.getInstance().setPose(new Pose2d(Drive.getInstance().getPose().getTranslation(),
                    Drive.getInstance().getPose().getRotation().plus(new Rotation2d(Math.PI))));
        }, Drive.getInstance()));

        operator.a().onTrue(Commands.runOnce(() -> {
            Drive.getInstance().stopWithX();
        }, Drive.getInstance()));

        operator.povUp().whileTrue(Commands.run(
                () -> IntakeWrist.getInstance().goToPosition(WristConstants.bumpPositionRotations),
                IntakeWrist.getInstance()));

        // Intake Controls
        // operator.leftTrigger().whileTrue(new InstantCommand(() -> {
        // Intake.getInstance().runVoltage(IntakeConstants.fastIntakeVoltage);
        // })).onFalse(new InstantCommand(() -> {
        // if (!IntakeWrist.getInstance().isStowed()) {
        // Intake.getInstance().runVoltage(IntakeConstants.slowIntakeVoltage);
        // } else {
        // Intake.getInstance().runVoltage(0);
        // }
        // }));

        // operator.povUp().onTrue(new InstantCommand(() -> {
        // IntakeWrist.getInstance().runVoltage(-WristConstants.manualVoltage);
        // }).andThen(new WaitCommand(0.8)).andThen(new InstantCommand(() -> {
        // IntakeWrist.getInstance().runVoltage(0);
        // }))).onFalse(new InstantCommand(() -> {
        // IntakeWrist.getInstance().runVoltage(WristConstants.manualVoltage);
        // Intake.getInstance().runVoltage(0);
        // }).andThen(new WaitCommand(0.8)).andThen(new InstantCommand(() -> {
        // IntakeWrist.getInstance().runVoltage(0);
        // }))).whileTrue(new RunCommand(() -> {
        // Intake.getInstance().runVoltage(IntakeConstants.fastIntakeVoltage);
        // }));

        // Shooter Controls
        operator.x().onTrue(
                Commands.runOnce(() -> {
                    RobotContainer.shooterCommand.setAutoShooting(true);
                }))
                .onFalse(
                        Commands.runOnce(() -> {
                            RobotContainer.shooterCommand.reset();
                        }));

        // Aim middle
        operator.leftBumper().onTrue(Commands.runOnce(() -> {
            RobotContainer.shooterCommand.shootMiddle();
        })).onFalse(Commands.runOnce(() -> {
            RobotContainer.shooterCommand.reset();
        }));

        // Aim near
        operator.rightBumper().onTrue(Commands.runOnce(() -> {
            RobotContainer.shooterCommand.shootNear();
        })).onFalse(Commands.runOnce(() -> {
            RobotContainer.shooterCommand.reset();
        }));

        // Pass
        // operator.rightTrigger().onTrue(Commands.runOnce(() -> {
        // RobotContainer.shooterCommand.nearPass();
        // })).onFalse(Commands.runOnce(() -> {
        // RobotContainer.shooterCommand.reset();
        // }));
        operator.rightTrigger().onTrue(Commands.runOnce(() -> {
            RobotContainer.shooterCommand.shootTower();
        })).onFalse(Commands.runOnce(() -> {
            RobotContainer.shooterCommand.reset();
        }));

        operator.leftTrigger().onTrue(Commands.runOnce(() -> {
            RobotContainer.shooterCommand.farPass();
        })).onFalse(Commands.runOnce(() -> {
            RobotContainer.shooterCommand.reset();
        }));

        // // ORCHESTRA STUFF II
        // operator.povUp().onTrue(new InstantCommand(() -> {
        // if (OrchestraUtils.isPlaying) {
        // if (OrchestraUtils.songPlaying == OrchestraUtils.songSelected) {
        // SmartDashboard.putString("DB/String 0",
        // "Paused: " + OrchestraUtils.allSongs[OrchestraUtils.songSelected]);
        // OrchestraUtils.orchestra.pause();
        // OrchestraUtils.isPlaying = false;
        // } else {
        // OrchestraUtils.orchestra.stop();
        // SmartDashboard.putString("DB/String 0",
        // "Playing: " + OrchestraUtils.allSongs[OrchestraUtils.songSelected]);
        // OrchestraUtils.orchestra
        // .loadMusic("music/" + OrchestraUtils.allSongs[OrchestraUtils.songSelected]);
        // OrchestraUtils.songPlaying = OrchestraUtils.songSelected;
        // OrchestraUtils.orchestra.play();
        // }
        // } else {
        // if (OrchestraUtils.songPlaying != OrchestraUtils.songSelected) {
        // OrchestraUtils.orchestra
        // .loadMusic("music/" + OrchestraUtils.allSongs[OrchestraUtils.songSelected]);
        // OrchestraUtils.songPlaying = OrchestraUtils.songSelected;
        // }
        // SmartDashboard.putString("DB/String 0",
        // "Playing: " + OrchestraUtils.allSongs[OrchestraUtils.songSelected]);
        // OrchestraUtils.isPlaying = true;
        // OrchestraUtils.orchestra.play();
        // }
        // }));
        // operator.povDown().onTrue(new InstantCommand(() -> {
        // if (OrchestraUtils.isPlaying) {
        // OrchestraUtils.orchestra.stop();
        // OrchestraUtils.isPlaying = false;
        // }
        // SmartDashboard.putString("DB/String 0", "Playing: ");
        // OrchestraUtils.songPlaying = -1;
        // }));
        // operator.povLeft().onTrue(new InstantCommand(() -> {
        // OrchestraUtils.songSelected = (OrchestraUtils.songSelected - 1 +
        // OrchestraUtils.allSongs.length)
        // % OrchestraUtils.allSongs.length;
        // SmartDashboard.putString("DB/String 1",
        // "Selected: " + OrchestraUtils.allSongs[OrchestraUtils.songSelected]);
        // }));
        // operator.povRight().onTrue(new InstantCommand(() -> {
        // OrchestraUtils.songSelected = (OrchestraUtils.songSelected + 1) %
        // OrchestraUtils.allSongs.length;
        // SmartDashboard.putString("DB/String 1",
        // "Selected: " + OrchestraUtils.allSongs[OrchestraUtils.songSelected]);
        // }));
    }
}
