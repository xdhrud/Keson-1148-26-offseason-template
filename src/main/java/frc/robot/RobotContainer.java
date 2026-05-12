// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import java.util.function.DoubleSupplier;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFX;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ShooterCommand;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.DriveConstants.FieldConstants;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.drive.ModuleIOTalonFXReal;
import frc.robot.subsystems.drive.ModuleIOTalonFXSim;
import frc.robot.subsystems.funnel.Funnel;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.wrist.IntakeWrist;
import frc.robot.subsystems.wrist.WristConstants;
import frc.robot.util.OrchestraUtils;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /**
     * This defines the runtime mode used by AdvantageKit. The mode is always "real"
     * when running on a
     * roboRIO. Change the value of "simMode" to switch between "sim" (physics sim)
     * and "replay" (log
     * replay from a file).
     */
    public static final Mode simMode = Mode.SIM;

    public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

    public static enum Mode {
        /** Running on a real robot. */
        REAL,

        /** Running a physics simulator. */
        SIM,

        /** Replaying from a log file. */
        REPLAY
    }

    // Subsystems
    public final Drive drive;
    public final Funnel funnel;
    public final Intake intake;
    public final Shooter shooter;
    public final Hood hood;
    public final IntakeWrist intakeWrist;
    public final Vision vision;

    // Commands
    public static ShooterCommand shooterCommand;

    // Controllers
    public final CommandXboxController operator = new CommandXboxController(1);
    public final CommandPS5Controller driver = new CommandPS5Controller(0);

    // Dashboard inputs
    private final LoggedDashboardChooser<Command> autoChooser;
    private final LoggedDashboardChooser<Command> songChooser;

    public static SwerveDriveSimulation driveSimulation = null;

    /**
     * The container for the robot. Contains subsystems, IO devices, and commands.
     */
    public RobotContainer() {
        switch (currentMode) {
            case REAL:
                // Real robot: instantiate hardware IO implementations
                vision = Vision.getInstance();
                drive = new Drive(
                        new GyroIOPigeon2(),
                        new ModuleIOTalonFXReal(DriveConstants.FrontLeft),
                        new ModuleIOTalonFXReal(DriveConstants.FrontRight),
                        new ModuleIOTalonFXReal(DriveConstants.BackLeft),
                        new ModuleIOTalonFXReal(DriveConstants.BackRight),
                        pose -> {
                        },
                        vision);
                funnel = Funnel.getInstance();
                intake = Intake.getInstance();
                shooter = Shooter.getInstance();
                hood = Hood.getInstance();
                intakeWrist = IntakeWrist.getInstance();
                break;

            case SIM:
                // Sim robot: instantiate physics sim IO implementations
                vision = Vision.getInstance();
                driveSimulation = new SwerveDriveSimulation(DriveConstants.mapleSimConfig,
                        new Pose2d(3, 3, new Rotation2d()));
                SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);
                drive = new Drive(
                        new GyroIOSim(driveSimulation.getGyroSimulation()),
                        new ModuleIOTalonFXSim(DriveConstants.FrontLeft, driveSimulation.getModules()[0]),
                        new ModuleIOTalonFXSim(DriveConstants.FrontRight, driveSimulation.getModules()[1]),
                        new ModuleIOTalonFXSim(DriveConstants.BackLeft, driveSimulation.getModules()[2]),
                        new ModuleIOTalonFXSim(DriveConstants.BackRight, driveSimulation.getModules()[3]),
                        driveSimulation::setSimulationWorldPose,
                        vision);
                funnel = Funnel.getInstance();
                intake = Intake.getInstance();
                shooter = Shooter.getInstance();
                hood = Hood.getInstance();
                intakeWrist = IntakeWrist.getInstance();
                break;

            default:
                // Replayed robot: disable IO implementations
                vision = Vision.getInstance();
                drive = new Drive(
                        new GyroIO() {
                        },
                        new ModuleIOTalonFX(DriveConstants.FrontLeft) {
                        },
                        new ModuleIOTalonFX(DriveConstants.FrontRight) {
                        },
                        new ModuleIOTalonFX(DriveConstants.BackLeft) {
                        },
                        new ModuleIOTalonFX(DriveConstants.BackRight) {
                        },
                        pose -> {
                        },
                        vision);
                funnel = Funnel.getInstance();
                intake = Intake.getInstance();
                shooter = Shooter.getInstance();
                hood = Hood.getInstance();
                intakeWrist = IntakeWrist.getInstance();
                break;
        }

        // Set up named commands for pathplanner
        NamedCommands.registerCommand("ShootNear", Commands.runOnce(() -> {
            shooterCommand.shootNear();
            shooterCommand.stopFunnel();
        }).andThen(Commands.waitUntil(() -> shooterCommand.isReady())).andThen(Commands.runOnce(() -> {
            shooterCommand.runFunnel();
        })));
        NamedCommands.registerCommand("ShootMiddle", Commands.runOnce(() -> {
            shooterCommand.shootMiddle();
            shooterCommand.stopFunnel();
        }).andThen(Commands.waitUntil(() -> shooterCommand.isReady())).andThen(Commands.runOnce(() -> {
            shooterCommand.runFunnel();
        })));
        NamedCommands.registerCommand("ShootTrench", Commands.runOnce(() -> {
            shooterCommand.shootTrench();
            shooterCommand.stopFunnel();
        }).andThen(Commands.waitUntil(() -> shooterCommand.isReady())).andThen(Commands.runOnce(() -> {
            shooterCommand.runFunnel();
        })));
        NamedCommands.registerCommand("StopShoot", Commands.runOnce(() -> {
            shooterCommand.reset();
        }));
        NamedCommands.registerCommand("DeployIntake", Commands.runOnce(() -> {
            intakeWrist.goToPosition(WristConstants.deployedPositionRotations);
            intake.runVoltage(IntakeConstants.fastIntakeVoltage);
        }, intakeWrist, intake));
        NamedCommands.registerCommand("LiftIntake", Commands.runOnce(() -> {
            intakeWrist.goToPosition(WristConstants.stowedPositionRotations);
            intake.runVoltage(IntakeConstants.slowIntakeVoltage);
        }, intakeWrist, intake));
        NamedCommands.registerCommand("Stop", Commands.runOnce(() -> {
            drive.stop();
        }, drive));
        NamedCommands.registerCommand("AutoRotate", Commands.run(() -> {
            DoubleSupplier rotateToHub = () -> {
                Pose2d pos = Drive.getInstance().getPose().exp(Drive.getInstance().getChassisSpeeds().toTwist2d(ShooterConstants.sotmPhaseShiftTimeSeconds));
                Pose2d hubPos = DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red
                        ? FieldConstants.RED_HUB_POS
                        : FieldConstants.BLUE_HUB_POS;
                double d = pos.getTranslation().getDistance(hubPos.getTranslation());
                double t = 0;
                Pose2d newPos;

                // Optimize until close enough
                do {
                    t = ShooterConstants.timeOfFlightMap.get(d);
        
                    ChassisSpeeds speeds = Drive.getInstance().getChassisSpeeds();
                    newPos = pos.plus(new Transform2d(speeds.vxMetersPerSecond * t, speeds.vyMetersPerSecond * t, new Rotation2d(speeds.omegaRadiansPerSecond * t)));
                    d = newPos.getTranslation().getDistance(hubPos.getTranslation());
                } while (Math.abs(t - ShooterConstants.timeOfFlightMap.get(d)) > ShooterConstants.sotmOptimizationTimeThresholdSeconds);

                Translation2d hub = hubPos.getTranslation();
                Translation2d robotToHub = hub.minus(newPos.getTranslation());
                return robotToHub.getAngle().getRadians() + Math.PI;
            };
            Drive.getInstance().autoRotate(new ChassisSpeeds(), new Rotation2d(rotateToHub.getAsDouble()));
        }));

        // Set up auto routines
        autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

        // Set up SysId routines
        autoChooser.addOption("Drive Wheel Radius Characterization",
                Drive.wheelRadiusCharacterization(drive));
        autoChooser.addOption("Drive Simple FF Characterization",
                Drive.feedforwardCharacterization(drive));
        autoChooser.addOption("Drive SysId (Quasistatic Forward)",
                drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption("Drive SysId (Quasistatic Reverse)",
                drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        autoChooser.addOption("Drive SysId (Dynamic Forward)",
                drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption("Drive SysID (Dynamic Reverse)",
                drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

        songChooser = new LoggedDashboardChooser<>("Orchestra Song Chooser");
        for (int song = 0; song < OrchestraUtils.allSongs.length; song++) {
            final int song1 = song;
            songChooser.addOption(OrchestraUtils.allSongs[song], Commands.runOnce(() -> {
                OrchestraUtils.songSelected = song1;
                SmartDashboard.putString("DB/String 1", "Selected: " + OrchestraUtils.allSongs[song1]);
            }));
        }
        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link GenericHID} or one of its subclasses
     * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
     * passing it to a
     * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        // normal field-relative drive
        drive.setDefaultCommand(
                DriveCommands.bigDrive(drive, () -> -driver.getLeftY(), () -> -driver.getLeftX(),
                        () -> -(driver.getRightX() < 0 ? -(Math.pow(Math.abs(driver.getRightX()), 1.5))
                                : Math.pow(driver.getRightX(), 1.5)),
                        () -> shooterCommand.isAutoRotating()));

        shooterCommand = new ShooterCommand();
        shooter.setDefaultCommand(shooterCommand);

        // Configure orchestra
        // configureOrchestra();

        // Assign controls in ControlMap
        ControlMap.getInstance().configurePreset1(operator, driver);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.get();
    }

    public void resetSimulationField() {
        if (currentMode != Mode.SIM)
            return;

        driveSimulation.setSimulationWorldPose(new Pose2d(3, 3, new Rotation2d()));
        SimulatedArena.getInstance().resetFieldForAuto();
    }

    public void updateSimulation() {
        if (currentMode != Mode.SIM)
            return;

        SimulatedArena.getInstance().simulationPeriodic();
        Logger.recordOutput("FieldSimulation/RobotPosition",
                driveSimulation.getSimulatedDriveTrainPose());
        Logger.recordOutput("FieldSimulation/Fuel",
                SimulatedArena.getInstance().getGamePiecesArrayByType("Fuel"));
    }

    public void configureOrchestra() {
        // ORCHESTRA STUFF
        int driveMotorIds[] = {};
        int rioMotorIds[] = { 18, 19 };
        for (int id : driveMotorIds) {
            OrchestraUtils.orchestra.addInstrument(new TalonFX(id, new CANBus("subsystems")));
        }
        for (int id : rioMotorIds) {
            OrchestraUtils.orchestra
                    .addInstrument(new TalonFX(id, new CANBus(DriveConstants.DrivetrainConstants.CANBusName)));
        }
        SmartDashboard.putString("DB/String 0", "Playing: ");
        SmartDashboard.putString("DB/String 1", "Selected: " + OrchestraUtils.allSongs[0]);
        OrchestraUtils.orchestra.loadMusic("music/" + OrchestraUtils.allSongs[0]);
    }
}