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

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.Consumer;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.DriveConstants.FieldConstants;
import frc.robot.subsystems.vision.CameraIO.TimestampedPose;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.util.LocalADStarAK;

public class Drive extends SubsystemBase {
    private static Drive instance;

    public static Drive getInstance() {
        return instance;
    }

    private final ProfiledPIDController rotationController;
    private final ProfiledPIDController positionXController;
    private final ProfiledPIDController positionYController;

    public final Field2d field = new Field2d();

    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
    private final Module[] modules = new Module[4]; // FL, FR, BL, BR
    private final SysIdRoutine sysId;
    private final Alert gyroDisconnectedAlert = new Alert("Disconnected gyro, using kinematics as fallback.",
            AlertType.kError);

    private final Vision vision;
    @AutoLogOutput
    private boolean visionActive = true;

    public boolean isVisionActive() {
        return visionActive;
    }

    public void setVisionActive(boolean visionActive) {
        this.visionActive = visionActive;
    }

    static final Lock odometryLock = new ReentrantLock();

    private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(Drive.getModuleTranslations());
    private Rotation2d rawGyroRotation = new Rotation2d();
    private SwerveModulePosition[] lastModulePositions = // For delta tracking
            new SwerveModulePosition[] {
                    new SwerveModulePosition(),
                    new SwerveModulePosition(),
                    new SwerveModulePosition(),
                    new SwerveModulePosition()
            };
    private SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(kinematics, rawGyroRotation,
            lastModulePositions, new Pose2d());

    // Discretization time constant
    private static final double DISCRETIZATION_TIME_SECONDS = 0.02;

    // Vision constants
    private static final double MAX_YAW_RATE_DEGREES_PER_SEC = 520.0;

    @AutoLogOutput
    private double sdMultiplier = 1.0;

    public void setSdMultiplier(double sdMultiplier) {
        this.sdMultiplier = sdMultiplier;
    }

    private final Consumer<Pose2d> resetSimulationPoseCallback;

    private double lastFieldUpdate = 0;

    public Drive(
            GyroIO gyroIO,
            ModuleIOTalonFX flModuleIO,
            ModuleIOTalonFX frModuleIO,
            ModuleIOTalonFX blModuleIO,
            ModuleIOTalonFX brModuleIO,
            Consumer<Pose2d> resetSimulationPoseCallback, Vision vision) {
        this.gyroIO = gyroIO;
        this.resetSimulationPoseCallback = resetSimulationPoseCallback;
        this.vision = vision;
        modules[0] = new Module(flModuleIO, 0, DriveConstants.FrontLeft);
        modules[1] = new Module(frModuleIO, 1, DriveConstants.FrontRight);
        modules[2] = new Module(blModuleIO, 2, DriveConstants.BackLeft);
        modules[3] = new Module(brModuleIO, 3, DriveConstants.BackRight);

        rotationController = new ProfiledPIDController(DriveConstants.kAngleP, DriveConstants.kAngleI,
                DriveConstants.kAngleD, new TrapezoidProfile.Constraints(DriveConstants.kMaxAngularSpeed,
                        DriveConstants.kMaxAngularAcceleration));
        rotationController.enableContinuousInput(-Math.PI, Math.PI);
        rotationController.setTolerance(Units.degreesToRadians(1));

        positionXController = new ProfiledPIDController(DriveConstants.kDriveP, DriveConstants.kDriveI,
                DriveConstants.kDriveD,
                new TrapezoidProfile.Constraints(DriveConstants.kMaxLinearSpeed.in(MetersPerSecond),
                        DriveConstants.kMaxLinearAcceleration.in(MetersPerSecond)));
        positionXController.setTolerance(0.1, 0.05);

        positionYController = new ProfiledPIDController(DriveConstants.kDriveP, DriveConstants.kDriveI,
                DriveConstants.kDriveD,
                new TrapezoidProfile.Constraints(DriveConstants.kMaxLinearSpeed.in(MetersPerSecond),
                        DriveConstants.kMaxLinearAcceleration.in(MetersPerSecond)));
        positionYController.setTolerance(0.1, 0.05);

        // Usage reporting for swerve template
        HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_AdvantageKit);

        // Start odometry thread
        PhoenixOdometryThread.getInstance().start();

        // Configure AutoBuilder for PathPlanner
        AutoBuilder.configure(
                this::getPose,
                this::setPose,
                this::getChassisSpeeds,
                this::runVelocity,
                new PPHolonomicDriveController(
                        new PIDConstants(
                                DriveConstants.PP_TRANSLATION_P,
                                DriveConstants.PP_TRANSLATION_I,
                                DriveConstants.PP_TRANSLATION_D),
                        new PIDConstants(
                                DriveConstants.PP_ROTATION_P,
                                DriveConstants.PP_ROTATION_I,
                                DriveConstants.PP_ROTATION_D)),
                DriveConstants.PP_CONFIG,
                () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                this);
        Pathfinding.setPathfinder(new LocalADStarAK());
        PathPlannerLogging.setLogActivePathCallback(
                (activePath) -> {
                    Logger.recordOutput(
                            "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
                });
        PathPlannerLogging.setLogTargetPoseCallback(
                (targetPose) -> {
                    Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
                });

        // Configure SysId
        sysId = new SysIdRoutine(
                new SysIdRoutine.Config(
                        null,
                        null,
                        null,
                        (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
                new SysIdRoutine.Mechanism(
                        (voltage) -> runCharacterization(voltage.in(Volts)), null, this));

        // Set simulation pose
        setPose(new Pose2d(new Translation2d(), new Rotation2d()));

        // Log field
        SmartDashboard.putData("Field", field);

        // Prevents double initialization since this constructor is called directly in
        // RobotContainer
        Drive.instance = this;

        // Setup NetworkTables communication
        NetworkCommunicator.getInstance().init();
    }

    @Override
    public void periodic() {
        odometryLock.lock(); // Prevents odometry updates while reading data
        gyroIO.updateInputs(gyroInputs);
        Logger.processInputs("RealOutputs/Drive/Gyro", gyroInputs);
        for (var module : modules) {
            module.periodic();
        }
        odometryLock.unlock();
        updateOdometry();
        vision.setRobotOrientation(rawGyroRotation);
        // Stop moving when disabled
        if (DriverStation.isDisabled()) {
            for (var module : modules) {
                module.stop();
            }
            vision.setThrottle(100);
            vision.setUseMegaTag2(false);
        } else {
            vision.setThrottle(0);
            vision.setUseMegaTag2(true);
        }

        // Log empty setpoint states when disabled
        if (DriverStation.isDisabled()) {
            Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
            Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
        }

        Logger.recordOutput("Odometry/Velocity/LinearVelocity", getLinearVelocity());
        Logger.recordOutput("Odometry/Velocity/AngularVelocity", getAngularVelocity());

        TimestampedPose[] timestampedPoses = vision.getTimestampedPoses();

        boolean acceptingTags = false;

        for (TimestampedPose pose : timestampedPoses) {
            if (pose != null && shouldAcceptPose(pose.pose) && visionActive) {
                acceptingTags = true;
                addVisionMeasurement(
                        pose.pose,
                        pose.timestamp,
                        VecBuilder.fill(
                                pose.stdMultiplier * VisionConstants.xyStdDev,
                                pose.stdMultiplier * VisionConstants.xyStdDev,
                                pose.stdMultiplier * VisionConstants.rStdDev));
            }
        }

        SmartDashboard.putBoolean("AcceptingTags", acceptingTags);

        if (Timer.getFPGATimestamp() - lastFieldUpdate > 0.1) {
            field.setRobotPose(getPose());
            vision.updateElasticPose();
            lastFieldUpdate = Timer.getFPGATimestamp();
        }

        // if (DriverStation.isDisabled()) {
        // rotationController.reset(getPose().getRotation().getRadians());
        // }

        // Update gyro alert
        gyroDisconnectedAlert.set(
                !gyroInputs.connected && RobotContainer.currentMode != RobotContainer.Mode.SIM);
    }

    /**
     * Runs the drive at the desired velocity.
     *
     * @param speeds Chassis speeds in meters/sec
     */
    public void runVelocity(ChassisSpeeds speeds) {
        // speeds.omegaRadiansPerSecond = -speeds.omegaRadiansPerSecond;
        // Convert to discrete time for better accuracy
        ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, DISCRETIZATION_TIME_SECONDS);

        // Calculate module setpoints
        SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);

        // Enforce velocity limits
        SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, DriveConstants.kMaxLinearSpeed);

        // Log unoptimized setpoints and setpoint speeds
        Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
        Logger.recordOutput("SwerveChassisSpeeds/Setpoints", discreteSpeeds);

        // Apply optimization to prevent module flipping
        for (int i = 0; i < 4; i++) {
            setpointStates[i].optimize(modules[i].getAngle());
        }

        // Send setpoints to modules
        for (int i = 0; i < 4; i++) {
            modules[i].runSetpoint(setpointStates[i]);
        }

        // Log optimized setpoints
        Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
    }

    /**
     * Rotates the drive to the desired setpoint while maintaining x and y motion.
     *
     * @param rotation Target rotation of the drive.
     */
    public void autoRotate(ChassisSpeeds speeds, Rotation2d targetRotation) {
        double current = getRotation().getRadians();
        double target = targetRotation.getRadians();

        double omega = rotationController.calculate(current, target);

        runVelocity(new ChassisSpeeds(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, omega));
    }

    /**
     * Drives with PID. Blocking
     */
    public Command PIDDrive(Translation2d localPose) {
        Translation2d target = getPose().getTranslation().plus(localPose);

        return new RunCommand(() -> {
            double currentX = getPose().getTranslation().getX();
            double currentY = getPose().getTranslation().getY();

            var xSetpoint = positionXController.getSetpoint();
            var ySetpoint = positionYController.getSetpoint();

            double xVel = xSetpoint.velocity +
                    positionXController.calculate(currentX, target.getX());

            double yVel = ySetpoint.velocity +
                    positionYController.calculate(currentY, target.getY());

            runVelocity(new ChassisSpeeds(
                    xVel,
                    yVel,
                    getChassisSpeeds().omegaRadiansPerSecond));
        }, this)
                .beforeStarting(() -> {
                    positionXController.reset(getPose().getTranslation().getX());
                    positionYController.reset(getPose().getTranslation().getY());
                })
                .until(() -> positionXController.atGoal() && positionYController.atGoal())
                .andThen(new InstantCommand(this::stop));
    }

    /**
     * Rotates with PID. Blocking
     */
    public Command PIDRotate(Rotation2d angle) {
        double target = angle.getRadians();

        return new RunCommand(() -> {
            double current = getPose().getRotation().getRadians();

            var setpoint = rotationController.getSetpoint();

            double omega = setpoint.velocity +
                    rotationController.calculate(current, target);

            runVelocity(new ChassisSpeeds(
                    getChassisSpeeds().vxMetersPerSecond,
                    getChassisSpeeds().vyMetersPerSecond,
                    omega));
        }, this)
                .beforeStarting(() -> {
                    rotationController.reset(getPose().getRotation().getRadians());
                })
                .until(() -> rotationController.atGoal())
                .andThen(new InstantCommand(this::stop));
    }

    /** Stops the drive by setting zero chassis speeds. */
    public void stop() {
        runVelocity(new ChassisSpeeds());
    }

    /**
     * Stops the drive and turns the modules to an X arrangement to resist movement.
     * The modules will
     * return to their normal orientations the next time a nonzero velocity is
     * requested.
     */
    public void stopWithX() {
        Rotation2d[] headings = new Rotation2d[4];
        for (int i = 0; i < 4; i++) {
            headings[i] = getModuleTranslations()[i].getAngle();
        }
        kinematics.resetHeadings(headings);
        stop();
    }

    /**
     * Returns the module states (turn angles and drive velocities) for all modules.
     */
    @AutoLogOutput(key = "SwerveStates/Measured")
    private SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            states[i] = modules[i].getState();
        }
        return states;
    }

    /**
     * Returns the module positions (turn angles and drive positions) for all
     * modules.
     */
    private SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            positions[i] = modules[i].getPosition();
        }
        return positions;
    }

    /** Returns the measured chassis speeds of the robot. */
    @AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
    public ChassisSpeeds getChassisSpeeds() {
        return kinematics.toChassisSpeeds(getModuleStates());
    }

    /** Returns the current odometry pose. */
    @AutoLogOutput(key = "Odometry/Robot")
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    /** Returns the current odometry rotation. */
    public Rotation2d getRotation() {
        return getPose().getRotation();
    }

    /** Resets the current odometry pose. */
    public void setPose(Pose2d pose) {
        resetSimulationPoseCallback.accept(pose);
        gyroIO.resetYaw(pose.getRotation());

        poseEstimator.resetPosition(pose.getRotation(), getModulePositions(), pose);
    }

    /** Adds a new timestamped vision measurement. */
    public void addVisionMeasurement(
            Pose2d visionRobotPoseMeters,
            double timestampSeconds,
            Matrix<N3, N1> visionMeasurementStdDevs) {
        poseEstimator.addVisionMeasurement(
                visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
    }

    /** Returns the maximum linear speed in meters per sec. */
    public double getMaxLinearSpeedMetersPerSec() {
        return DriveConstants.kMaxLinearSpeed.in(MetersPerSecond);
    }

    /** Returns the maximum angular speed in radians per sec. */
    public double getMaxAngularSpeedRadPerSec() {
        return getMaxLinearSpeedMetersPerSec() / DriveConstants.DRIVE_BASE_RADIUS;
    }

    @AutoLogOutput(key = "Drive/DistanceFromHub")
    public double getDistanceFromHub() {
        Pose2d pos = poseEstimator.getEstimatedPosition();
        Pose2d hubPos = DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red
                ? FieldConstants.RED_HUB_POS
                : FieldConstants.BLUE_HUB_POS;
        return pos.getTranslation().getDistance(hubPos.getTranslation());
    }

    @AutoLogOutput(key = "Drive/InShootingRange")
    public boolean isInShootingRange() {
        boolean allianceIsRed = DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == Alliance.Red;
        return allianceIsRed ? poseEstimator.getEstimatedPosition().getX() > FieldConstants.RED_HUB_CENTER_X_METERS
                : poseEstimator.getEstimatedPosition().getX() < FieldConstants.BLUE_HUB_CENTER_X_METERS;
    }

    public boolean isInOpponentShootingRange() {
        boolean allianceIsRed = DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == Alliance.Red;
        return allianceIsRed ? poseEstimator.getEstimatedPosition().getX() < FieldConstants.BLUE_HUB_CENTER_X_METERS
                : poseEstimator.getEstimatedPosition().getX() > FieldConstants.RED_HUB_CENTER_X_METERS;
    }

    /** Returns an array of module translations. */
    public static Translation2d[] getModuleTranslations() {
        return new Translation2d[] {
                new Translation2d(DriveConstants.FrontLeft.LocationX, DriveConstants.FrontLeft.LocationY),
                new Translation2d(DriveConstants.FrontRight.LocationX, DriveConstants.FrontRight.LocationY),
                new Translation2d(DriveConstants.BackLeft.LocationX, DriveConstants.BackLeft.LocationY),
                new Translation2d(DriveConstants.BackRight.LocationX, DriveConstants.BackRight.LocationY)
        };
    }

    /**
     * Updates odometry using wheel positions and gyro data. Handles high-frequency
     * sampling and
     * provides comprehensive logging with performance optimization.
     */
    private void updateOdometry() {
        double[] sampleTimestamps = modules[0].getOdometryTimestamps();
        int sampleCount = sampleTimestamps.length;

        for (int i = 0; i < sampleCount; i++) {
            // Read current module positions
            SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
            SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];

            for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
                modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
                moduleDeltas[moduleIndex] = new SwerveModulePosition(
                        modulePositions[moduleIndex].distanceMeters
                                - lastModulePositions[moduleIndex].distanceMeters,
                        modulePositions[moduleIndex].angle);
                lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
            }

            // Update gyro rotation with fallback to kinematics
            updateGyroRotation(moduleDeltas, i);

            // Apply odometry update
            poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
        }
    }

    /**
     * Updates gyro rotation with fallback to kinematics-based estimation.
     *
     * @param moduleDeltas The change in module positions since last update
     * @param sampleIndex  The current sample index
     */
    private void updateGyroRotation(SwerveModulePosition[] moduleDeltas, int sampleIndex) {
        if (gyroInputs.connected) {
            // Use real gyro data when available
            rawGyroRotation = gyroInputs.odometryYawPositions[sampleIndex];
        } else {
            // Fall back to kinematics-based rotation estimation
            Twist2d twist = kinematics.toTwist2d(moduleDeltas);
            rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
        }
    }

    private double getLinearVelocity() {
        ChassisSpeeds speeds = getChassisSpeeds();
        double linearSpeed = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
        return linearSpeed;
    }

    private double getAngularVelocity() {
        return gyroInputs.yawVelocityRadPerSec;
    }

    /**
     * Determines if a vision pose should be accepted based on various criteria.
     *
     * @param pose The vision pose
     * @return True if the pose should be accepted, false otherwise
     */
    public boolean shouldAcceptPose(Pose2d pose) {
        // Always accept poses when disabled
        if (DriverStation.isDisabled()) {
            return true;
        }

        // Reject if rotating too quickly
        if (Math.abs(Units.radiansToDegrees(gyroInputs.yawVelocityRadPerSec)) >= MAX_YAW_RATE_DEGREES_PER_SEC) {
            return false;
        }

        // Reject if outside field bounds
        if (isOutsideFieldBounds(pose)) {
            return false;
        }

        // Accept the pose
        return true;
    }

    /** Checks if a pose is outside the field boundaries. */
    private boolean isOutsideFieldBounds(Pose2d pose) {
        return pose.getX() < -FieldConstants.FIELD_BORDER_MARGIN_METERS
                || pose.getX() > FieldConstants.FIELD_WIDTH_METERS + FieldConstants.FIELD_BORDER_MARGIN_METERS
                || pose.getY() < -FieldConstants.FIELD_BORDER_MARGIN_METERS
                || pose.getY() > FieldConstants.FIELD_HEIGHT_METERS + FieldConstants.FIELD_BORDER_MARGIN_METERS;
    }

    /**
     * Runs the drive in a straight line with the specified drive output. Used for
     * system
     * identification.
     *
     * @param output Voltage output to apply to all modules
     */
    public void runCharacterization(double output) {
        Logger.recordOutput("SwerveStates/VoltageSetpoint", output);
        for (int i = 0; i < 4; i++) {
            modules[i].runVoltage(output);
        }
    }

    // SysID Commands

    /** Returns a command to run a quasistatic test in the specified direction. */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return run(() -> runCharacterization(0.0))
                .withTimeout(1.0)
                .andThen(sysId.quasistatic(direction));
    }

    /** Returns a command to run a dynamic test in the specified direction. */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
    }

    // Wheel Radius Characterization
    private static final double WHEEL_RADIUS_MAX_VELOCITY = 30; // Rad/Sec
    private static final double WHEEL_RADIUS_RAMP_RATE = 6; // Rad/Sec^2

    /** Returns the position of each module in radians. */
    public double[] getWheelRadiusCharacterizationPositions() {
        double[] values = new double[4];
        for (int i = 0; i < 4; i++) {
            values[i] = modules[i].getWheelRadiusCharacterizationPosition();
        }
        return values;
    }

    private static class WheelRadiusCharacterizationState {
        double[] positions = new double[4];
        Rotation2d lastAngle = Rotation2d.kZero;
        double gyroDelta = 0.0;
    }

    /** Measures the robot's wheel radius by spinning in a circle. */
    public static Command wheelRadiusCharacterization(Drive drive) {
        SlewRateLimiter limiter = new SlewRateLimiter(WHEEL_RADIUS_RAMP_RATE);
        WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();

        return Commands.parallel(
                // Drive control sequence
                Commands.sequence(
                        // Reset acceleration limiter
                        Commands.runOnce(() -> limiter.reset(0.0)),

                        // Turn in place, accelerating up to full speed
                        Commands.run(
                                () -> {
                                    double speed = limiter.calculate(WHEEL_RADIUS_MAX_VELOCITY);
                                    drive.runVelocity(new ChassisSpeeds(0.0, 0.0, speed));
                                },
                                drive)),

                // Measurement sequence
                Commands.sequence(
                        // Wait for modules to fully orient before starting measurement
                        Commands.waitSeconds(1.0),

                        // Record starting measurement
                        Commands.runOnce(
                                () -> {
                                    state.positions = drive.getWheelRadiusCharacterizationPositions();
                                    state.lastAngle = drive.getRotation();
                                    state.gyroDelta = 0.0;
                                }),

                        // Update gyro delta
                        Commands.run(
                                () -> {
                                    var rotation = drive.getRotation();
                                    state.gyroDelta += Math.abs(rotation.minus(state.lastAngle).getRadians());
                                    state.lastAngle = rotation;

                                    double[] positions = drive.getWheelRadiusCharacterizationPositions();
                                    double wheelDelta = 0.0;
                                    for (int i = 0; i < 4; i++) {
                                        wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
                                    }
                                    double wheelRadius = (state.gyroDelta * DriveConstants.DRIVE_BASE_RADIUS)
                                            / wheelDelta;

                                    Logger.recordOutput("Drive/WheelDelta", wheelDelta);
                                    Logger.recordOutput("Drive/WheelRadius", wheelRadius);
                                })

                                // When cancelled, calculate and print results
                                .finallyDo(
                                        () -> {
                                            double[] positions = drive.getWheelRadiusCharacterizationPositions();
                                            double wheelDelta = 0.0;
                                            for (int i = 0; i < 4; i++) {
                                                wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
                                            }
                                            double wheelRadius = (state.gyroDelta * DriveConstants.DRIVE_BASE_RADIUS)
                                                    / wheelDelta;

                                            NumberFormat formatter = new DecimalFormat(
                                                    "#0.000000000000000000000000000");
                                            System.out.println(
                                                    "********** Wheel Radius Characterization Results **********");
                                            System.out.println(
                                                    "\tWheel Delta: " + formatter.format(wheelDelta) + " radians");
                                            System.out.println(
                                                    "\tGyro Delta: " + formatter.format(state.gyroDelta) + " radians");
                                            System.out.println(
                                                    "\tWheel Radius: "
                                                            + formatter.format(wheelRadius)
                                                            + " meters, "
                                                            + formatter.format(Units.metersToInches(wheelRadius))
                                                            + " inches");
                                        })));
    }

    // Feedforward Characterization
    private static final double FF_START_DELAY = 2.0; // Secs
    private static final double FF_RAMP_RATE = 0.1; // Volts/Sec

    /** Returns the average velocity of the modules in rotations/sec. */
    public double getFFCharacterizationVelocity() {
        double totalVelocity = 0.0;
        for (int i = 0; i < 4; i++) {
            totalVelocity += modules[i].getFFCharacterizationVelocity();
        }
        return totalVelocity / 4.0;
    }

    /**
     * Measures the velocity feedforward constants for the drive motors.
     *
     * <p>
     * This command should only be used in voltage control mode.
     */
    public static Command feedforwardCharacterization(Drive drive) {
        List<Double> velocitySamples = new LinkedList<>();
        List<Double> voltageSamples = new LinkedList<>();
        Timer timer = new Timer();

        return Commands.sequence(
                // Reset data
                Commands.runOnce(
                        () -> {
                            velocitySamples.clear();
                            voltageSamples.clear();
                        }),

                // Allow modules to orient
                Commands.run(() -> drive.runCharacterization(0.0), drive).withTimeout(FF_START_DELAY),

                // Start timer
                Commands.runOnce(timer::restart),

                // Accelerate and gather data
                Commands.run(
                        () -> {
                            double voltage = timer.get() * FF_RAMP_RATE;
                            drive.runCharacterization(voltage);
                            velocitySamples.add(drive.getFFCharacterizationVelocity());
                            voltageSamples.add(voltage);
                        },
                        drive)

                        // When cancelled, calculate and print results
                        .finallyDo(
                                () -> {
                                    int n = velocitySamples.size();
                                    double sumX = 0.0;
                                    double sumY = 0.0;
                                    double sumXY = 0.0;
                                    double sumX2 = 0.0;
                                    for (int i = 0; i < n; i++) {
                                        sumX += velocitySamples.get(i);
                                        sumY += voltageSamples.get(i);
                                        sumXY += velocitySamples.get(i) * voltageSamples.get(i);
                                        sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
                                    }
                                    double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
                                    double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

                                    NumberFormat formatter = new DecimalFormat("#0.00000");
                                    System.out.println("********** Drive FF Characterization Results **********");
                                    System.out.println("\tkS: " + formatter.format(kS));
                                    System.out.println("\tkV: " + formatter.format(kV));
                                }));
    }
}