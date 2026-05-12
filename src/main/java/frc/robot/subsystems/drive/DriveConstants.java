package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.MountPoseConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.DriveMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;

public class DriveConstants {
    // The steer motor uses any SwerveModule.SteerRequestType control request with
    // the
    // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput

    // ================================= PID Tuning
    // =================================

    // Swerve Steer PID Values
    public static final double kSteerP = 45;
    public static final double kSteerI = 0;
    public static final double kSteerD = 0;
    public static final double kSteerS = 0;
    public static final double kSteerV = 0;
    public static final double kSteerA = 0;
    public static final StaticFeedforwardSignValue kStaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;

    // Swerve Drive PID Values
    public static final double kDriveP = 0.050653;
    public static final double kDriveI = 0;
    public static final double kDriveD = 0;
    public static final double kDriveS = 0.21122;
    public static final double kDriveV = 0.13669;
    public static final double kDriveA = 0.019654;

    // Swerve Angular PID Values (Position)
    public static final double kAngleP = 17;
    public static final double kAngleI = 0;
    public static final double kAngleD = 0;

    // Swerve Position PID Values
    public static final double kPositionP = 25;
    public static final double kPositionI = 0.1;
    public static final double kPositionD = 50;

    // The closed-loop output type to use for the motors;
    // This affects PID/FF gains
    public static final ClosedLoopOutputType kSteerClosedLoopOutput = ClosedLoopOutputType.Voltage;
    public static final ClosedLoopOutputType kDriveClosedLoopOutput = ClosedLoopOutputType.Voltage;

    // PathPlanner PIDs
    public static double PP_ROTATION_P = 50;
    public static double PP_ROTATION_I = 1.5;
    public static double PP_ROTATION_D = 2;
    public static double PP_TRANSLATION_P = 60;
    public static double PP_TRANSLATION_I = 0;
    public static double PP_TRANSLATION_D = 0;

    // ================================= Hardware Tuning
    // =================================

    public static final double ROBOT_MASS_KG = 62.20248248; // KG, NOT LB
    public static final double ROBOT_MOI = 6.4848130711; // KG*M^2 , NOT LB*IN^2
    public static final double WHEEL_COF = 1.2;
    public static final double ROBOT_LENGTH_X_BUMPER = 33.75;
    public static final double ROBOT_LENGTH_Y_BUMPER = 33.875;

    // The type of motor used for the drive motor
    public static final DriveMotorArrangement kDriveMotorType = DriveMotorArrangement.TalonFX_Integrated;
    // The type of motor used for the steer motor
    public static final SteerMotorArrangement kSteerMotorType = SteerMotorArrangement.TalonFX_Integrated;

    // The remote sensor feedback type to use for the steer motors;
    // When not Pro-licensed, FusedCANcoder/SyncCANcoder automatically fall back to
    // RemoteCANcoder
    public static final SteerFeedbackType kSteerFeedbackType = SteerFeedbackType.FusedCANcoder;

    // The stator current at which the wheels start to slip;
    public static final Current kSlipCurrent = Amps.of(60);

    // Theoretical linear free speed (m/s) at 12 V applied output;
    public static final LinearVelocity kMaxLinearSpeed = MetersPerSecond.of(30); // 3.95
    public static final LinearVelocity kMaxLinearAcceleration = MetersPerSecond.of(20.0);

    // Theoretical rotational free speed (m/s) at 12 V applied output;
    public static final double kMaxAngularSpeed = 50; // 9.35 real speed
    public static final double kMaxAngularAcceleration = 18.0;

    // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
    public static final double kCoupleRatio = 3.375;

    public static final double kDriveGearRatio = 7.03;
    public static final double kSteerGearRatio = 287.0 / 11.0;
    public static final Distance kWheelRadius = Inches.of(1.962754845085376500000000000);

    public static final boolean kInvertLeftSide = false;
    public static final boolean kInvertRightSide = true;

    public static final int kPigeonId = 30;

    // Simulated moment of inertia for the steer and drive motors;
    public static final MomentOfInertia kSteerInertia = KilogramSquareMeters.of(0.01);
    public static final MomentOfInertia kDriveInertia = KilogramSquareMeters.of(0.01);
    // Simulated voltage necessary to overcome friction
    public static final Voltage kSteerFrictionVoltage = Volts.of(0.2);
    public static final Voltage kDriveFrictionVoltage = Volts.of(0.2);

    // Front Left
    private static final int kFrontLeftDriveMotorId = 5;
    private static final int kFrontLeftSteerMotorId = 6;
    private static final int kFrontLeftEncoderId = 11;
    private static final Angle kFrontLeftEncoderOffset = Rotations.of(-0.142333984375);
    private static final boolean kFrontLeftSteerMotorInverted = false;
    private static final boolean kFrontLeftEncoderInverted = false;

    private static final Distance kFrontLeftEncoderXPos = Inches.of(10.875);
    private static final Distance kFrontLeftEncoderYPos = Inches.of(10.875);

    // Front Right
    private static final int kFrontRightDriveMotorId = 7;
    private static final int kFrontRightSteerMotorId = 8;
    private static final int kFrontRightEncoderId = 12;
    private static final Angle kFrontRightEncoderOffset = Rotations.of(-0.315185546875);
    private static final boolean kFrontRightSteerMotorInverted = false;
    private static final boolean kFrontRightEncoderInverted = false;

    private static final Distance kFrontRightEncoderXPos = Inches.of(10.875);
    private static final Distance kFrontRightEncoderYPos = Inches.of(-10.875);

    // Back Left
    private static final int kBackLeftDriveMotorId = 1;
    private static final int kBackLeftSteerMotorId = 2;
    private static final int kBackLeftEncoderId = 9;
    private static final Angle kBackLeftEncoderOffset = Rotations.of(-0.21142578125 + 0.5);
    private static final boolean kBackLeftSteerMotorInverted = false;
    private static final boolean kBackLeftEncoderInverted = false;

    private static final Distance kBackLeftEncoderXPos = Inches.of(-10.875);
    private static final Distance kBackLeftEncoderYPos = Inches.of(10.875);

    // Back Right
    private static final int kBackRightDriveMotorId = 3;
    private static final int kBackRightSteerMotorId = 4;
    private static final int kBackRightEncoderId = 10;
    private static final Angle kBackRightEncoderOffset = Rotations.of(0.29736328125 + 0.5);
    private static final boolean kBackRightSteerMotorInverted = false;
    private static final boolean kBackRightEncoderInverted = false;

    private static final Distance kBackRightEncoderXPos = Inches.of(-10.875);
    private static final Distance kBackRightEncoderYPos = Inches.of(-10.875);

    public static final double DRIVE_BASE_RADIUS = Math.max(
            Math.max(
                    Math.hypot(kFrontLeftEncoderXPos.baseUnitMagnitude(), kFrontLeftEncoderYPos.baseUnitMagnitude()),
                    Math.hypot(kFrontRightEncoderXPos.baseUnitMagnitude(), kFrontRightEncoderYPos.baseUnitMagnitude())),
            Math.max(
                    Math.hypot(kBackLeftEncoderXPos.baseUnitMagnitude(), kBackLeftEncoderYPos.baseUnitMagnitude()),
                    Math.hypot(kBackRightEncoderXPos.baseUnitMagnitude(), kBackRightEncoderYPos.baseUnitMagnitude())));

    // Gyro offsets
    public static final double gyroYawOffset = RobotBase.isSimulation() ? 0 : 180;
    public static final double gyroPitchOffset = 0.615234 ;
    public static final double gyroRollOffset = 0.175 ;

    // ================================= FIELD CONSTANTS
    // =================================

    public static class FieldConstants {
        public static final double FIELD_WIDTH_METERS = 16.513048;
        public static final double FIELD_HEIGHT_METERS = 8.042656;
        public static final double FIELD_BORDER_MARGIN_METERS = 0.05;

        public static final double BLUE_HUB_CENTER_X_METERS = 4.63;
        public static final double BLUE_HUB_CENTER_Y_METERS = 4.035;

        public static final double RED_HUB_CENTER_X_METERS = 11.92;
        public static final double RED_HUB_CENTER_Y_METERS = 4.035;

        public static final double HUB_HEIGHT_METERS = 1.06;

        public static final Pose2d RED_HUB_POS = new Pose2d(RED_HUB_CENTER_X_METERS, RED_HUB_CENTER_Y_METERS,
                new Rotation2d());
        public static final Pose2d BLUE_HUB_POS = new Pose2d(BLUE_HUB_CENTER_X_METERS, BLUE_HUB_CENTER_Y_METERS,
                new Rotation2d());
        public static final Pose2d FIELD_CENTER_POS = new Pose2d(FIELD_WIDTH_METERS / 2, FIELD_HEIGHT_METERS / 2,
                new Rotation2d());
    }

    // ================================= END OF TUNER CONSTANTS
    // =================================

    // ================================= Robot Software Configs
    // =================================

    public static final Slot0Configs steerGains = new Slot0Configs()
            .withKP(kSteerP)
            .withKI(kSteerI)
            .withKD(kSteerD)
            .withKS(kSteerS)
            .withKV(kSteerV)
            .withKA(kSteerA)
            .withStaticFeedforwardSign(kStaticFeedforwardSign);
    // When using closed-loop control, the drive motor uses the control
    // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
    public static final Slot0Configs driveGains = new Slot0Configs()
            .withKP(kDriveP)
            .withKI(kDriveI)
            .withKD(kDriveD)
            .withKS(kDriveS)
            .withKV(kDriveV)
            .withKA(kDriveA);

    // Initial configs for the drive and steer motors and the azimuth encoder; these
    // cannot be null.
    // Some configs will be overwritten; check the `with*InitialConfigs()` API
    // documentation.
    public static final TalonFXConfiguration driveInitialConfigs = new TalonFXConfiguration()
            .withCurrentLimits(
                    new CurrentLimitsConfigs()
                            .withSupplyCurrentLimitEnable(true)
                            .withSupplyCurrentLimit(Amps.of(40))
                            .withStatorCurrentLimitEnable(true)
                            .withStatorCurrentLimit(kSlipCurrent))
            .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));
    public static final TalonFXConfiguration steerInitialConfigs = new TalonFXConfiguration()
            .withCurrentLimits(
                    new CurrentLimitsConfigs()
                            // Swerve azimuth does not require much torque output, so we can set a
                            // relatively low stator current limit to help avoid brownouts without
                            // impacting performance.
                            .withStatorCurrentLimit(Amps.of(30))
                            .withStatorCurrentLimitEnable(true))
            .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast));

    public static final CANcoderConfiguration encoderInitialConfigs = new CANcoderConfiguration();

    // Configs for the Pigeon 2; leave this null to skip applying Pigeon 2 configs
    public static final Pigeon2Configuration pigeonConfigs = new Pigeon2Configuration()
            .withMountPose(new MountPoseConfigs().withMountPoseYaw(gyroYawOffset).withMountPosePitch(gyroPitchOffset)
                    .withMountPoseRoll(gyroRollOffset));

    // CAN bus that the devices are located on;
    // All swerve devices must share the same CAN bus
    public static final CANBus kCANBus = new CANBus("rio", "./logs/driveCAN.hoot");

    public static final SwerveDrivetrainConstants DrivetrainConstants = new SwerveDrivetrainConstants()
            .withCANBusName(kCANBus.getName())
            .withPigeon2Id(kPigeonId)
            .withPigeon2Configs(pigeonConfigs);

    // ================================= Module Configurations
    // =================================

    public static final SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> ConstantCreator = new SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
            .withDriveMotorGearRatio(kDriveGearRatio)
            .withSteerMotorGearRatio(kSteerGearRatio)
            .withCouplingGearRatio(kCoupleRatio)
            .withWheelRadius(kWheelRadius)
            .withSteerMotorGains(steerGains)
            .withDriveMotorGains(driveGains)
            .withSteerMotorClosedLoopOutput(kSteerClosedLoopOutput)
            .withDriveMotorClosedLoopOutput(kDriveClosedLoopOutput)
            .withSlipCurrent(kSlipCurrent)
            .withSpeedAt12Volts(kMaxLinearSpeed)
            .withDriveMotorType(kDriveMotorType)
            .withSteerMotorType(kSteerMotorType)
            .withFeedbackSource(kSteerFeedbackType)
            .withDriveMotorInitialConfigs(driveInitialConfigs)
            .withSteerMotorInitialConfigs(steerInitialConfigs)
            .withEncoderInitialConfigs(encoderInitialConfigs)
            .withSteerInertia(kSteerInertia)
            .withDriveInertia(kDriveInertia)
            .withSteerFrictionVoltage(kSteerFrictionVoltage)
            .withDriveFrictionVoltage(kDriveFrictionVoltage);

    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FrontLeft = ConstantCreator
            .createModuleConstants(
                    DriveConstants.kFrontLeftSteerMotorId,
                    DriveConstants.kFrontLeftDriveMotorId,
                    DriveConstants.kFrontLeftEncoderId,
                    DriveConstants.kFrontLeftEncoderOffset,
                    DriveConstants.kFrontLeftEncoderXPos,
                    DriveConstants.kFrontLeftEncoderYPos,
                    DriveConstants.kInvertLeftSide,
                    DriveConstants.kFrontLeftSteerMotorInverted,
                    DriveConstants.kFrontLeftEncoderInverted)
            .withSlipCurrent(DriveConstants.kSlipCurrent);
    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FrontRight = ConstantCreator
            .createModuleConstants(
                    DriveConstants.kFrontRightSteerMotorId,
                    DriveConstants.kFrontRightDriveMotorId,
                    DriveConstants.kFrontRightEncoderId,
                    DriveConstants.kFrontRightEncoderOffset,
                    DriveConstants.kFrontRightEncoderXPos,
                    DriveConstants.kFrontRightEncoderYPos,
                    DriveConstants.kInvertRightSide,
                    DriveConstants.kFrontRightSteerMotorInverted,
                    DriveConstants.kFrontRightEncoderInverted)
            .withSlipCurrent(DriveConstants.kSlipCurrent);
    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BackLeft = ConstantCreator
            .createModuleConstants(
                    DriveConstants.kBackLeftSteerMotorId,
                    DriveConstants.kBackLeftDriveMotorId,
                    DriveConstants.kBackLeftEncoderId,
                    DriveConstants.kBackLeftEncoderOffset,
                    DriveConstants.kBackLeftEncoderXPos,
                    DriveConstants.kBackLeftEncoderYPos,
                    DriveConstants.kInvertLeftSide,
                    DriveConstants.kBackLeftSteerMotorInverted,
                    DriveConstants.kBackLeftEncoderInverted)
            .withSlipCurrent(DriveConstants.kSlipCurrent);
    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BackRight = ConstantCreator
            .createModuleConstants(
                    DriveConstants.kBackRightSteerMotorId,
                    DriveConstants.kBackRightDriveMotorId,
                    DriveConstants.kBackRightEncoderId,
                    DriveConstants.kBackRightEncoderOffset,
                    DriveConstants.kBackRightEncoderXPos,
                    DriveConstants.kBackRightEncoderYPos,
                    DriveConstants.kInvertRightSide,
                    DriveConstants.kBackRightSteerMotorInverted,
                    DriveConstants.kBackRightEncoderInverted)
            .withSlipCurrent(DriveConstants.kSlipCurrent);

    // ================================= PathPlanner Configs
    // =================================

    public static final RobotConfig PP_CONFIG = new RobotConfig(
            DriveConstants.ROBOT_MASS_KG,
            DriveConstants.ROBOT_MOI,
            new ModuleConfig(
                    DriveConstants.kWheelRadius.baseUnitMagnitude(),
                    DriveConstants.kMaxLinearSpeed.in(MetersPerSecond),
                    DriveConstants.WHEEL_COF,
                    DCMotor.getKrakenX60Foc(1).withReduction(DriveConstants.kDriveGearRatio),
                    DriveConstants.kSlipCurrent.in(Amps),
                    1),
            Drive.getModuleTranslations());

    // ================================= Simulation Configs
    // =================================

    // Create and configure a drivetrain simulation configuration
    public static final DriveTrainSimulationConfig mapleSimConfig = DriveTrainSimulationConfig.Default()
            // Specify robot mass
            .withRobotMass(Kilograms.of(DriveConstants.ROBOT_MASS_KG)) // Set robot mass in kg
            // Specify gyro type (for realistic gyro drifting and error simulation)
            .withGyro(COTS.ofPigeon2())
            // Specify module positions
            .withCustomModuleTranslations(Drive.getModuleTranslations())
            // Specify swerve module (for realistic swerve dynamics)
            .withSwerveModule(
                    new SwerveModuleSimulationConfig(
                            DCMotor.getKrakenX60(1), // Drive motor is a Kraken X60
                            DCMotor.getFalcon500(1), // Steer motor is a Falcon 500
                            DriveConstants.kDriveGearRatio, // Drive motor gear ratio.
                            DriveConstants.kSteerGearRatio, // Steer motor gear ratio.
                            DriveConstants.kDriveFrictionVoltage, // Drive friction voltage.
                            DriveConstants.kSteerFrictionVoltage, // Steer friction voltage
                            Inches.of(DriveConstants.kWheelRadius.magnitude()), // Wheel radius
                            DriveConstants.kSteerInertia, // Steer MOI
                            WHEEL_COF)) // Wheel COF
            // Configures the track length and track width (spacing between swerve modules)
            .withTrackLengthTrackWidth(
                    Inches.of(
                            Math.abs(kFrontLeftEncoderXPos.magnitude()) + Math.abs(kFrontRightEncoderXPos.magnitude())),
                    Inches.of(Math.abs(kFrontLeftEncoderYPos.magnitude()) + Math.abs(kBackLeftEncoderYPos.magnitude())))
            // Configures the bumper size (dimensions of the robot bumper)
            .withBumperSize(Inches.of(ROBOT_LENGTH_X_BUMPER), Inches.of(ROBOT_LENGTH_Y_BUMPER));

    // ================================= Extra Configurations
    // =================================

    public static final double ODOMETRY_FREQUENCY = new CANBus(DriveConstants.DrivetrainConstants.CANBusName)
            .isNetworkFD() ? 250.0 : 100.0;
}
