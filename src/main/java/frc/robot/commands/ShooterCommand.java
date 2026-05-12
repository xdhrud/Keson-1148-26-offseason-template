package frc.robot.commands;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants.FieldConstants;
import frc.robot.subsystems.funnel.Funnel;
import frc.robot.subsystems.funnel.FunnelConstants;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hood.HoodConstants;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.util.LoggedTunableNumber;

public class ShooterCommand extends Command {
    @AutoLogOutput
    private double shooterVelocityRPS;
    @AutoLogOutput
    private double hoodAngleDeg;
    private double funnelVoltage;
    private double indexerVoltage;
    private boolean autoShooting;
    private boolean autoRotating;
    private boolean runningFunnel = false;
    private boolean passing = false;

    public LoggedTunableNumber tuneHoodAngleDeg = new LoggedTunableNumber("ShooterCommand/tuneHoodAngleDeg", HoodConstants.hoodAngleShootTower);
    public LoggedTunableNumber tuneShooterVelocityRPS = new LoggedTunableNumber("ShooterCommand/tuneShooterVelocityRPS", ShooterConstants.towerRPS);

    public boolean isRunningFunnel() {
        return runningFunnel;
    }

    public ShooterCommand() {
        addRequirements(Shooter.getInstance(), Hood.getInstance(), Funnel.getInstance());
        shooterVelocityRPS = ShooterConstants.defaultRPS;
        hoodAngleDeg = HoodConstants.maxAngleDegrees;
        funnelVoltage = FunnelConstants.funnelSlowVolts;
        indexerVoltage = FunnelConstants.indexerSlowVolts;
        autoShooting = false;
        autoRotating = false;
        passing = false;

        // Populate map
        ShooterConstants.hoodAngleMap.put(1.71974, HoodConstants.hoodAngleShootNear);
        ShooterConstants.shooterVelocityMap.put(1.71974, ShooterConstants.nearRPS);
        ShooterConstants.timeOfFlightMap.put(1.71974, 1.0);
        ShooterConstants.indexerVoltageMap.put(1.71974, 3.8);

        ShooterConstants.hoodAngleMap.put(2.17, 69.0);
        ShooterConstants.shooterVelocityMap.put(2.17, 48.0);
        ShooterConstants.timeOfFlightMap.put(2.17, 8.0);
        // ShooterConstants.indexerVoltageMap.put(2.65, 6.0);

        ShooterConstants.hoodAngleMap.put(2.65, 63.0);
        ShooterConstants.shooterVelocityMap.put(2.65, 48.0);
        ShooterConstants.timeOfFlightMap.put(2.65, 8.0);
        ShooterConstants.indexerVoltageMap.put(2.65, 5.0);

        ShooterConstants.hoodAngleMap.put(4.12, 57.0);
        ShooterConstants.shooterVelocityMap.put(4.12, 53.0);
        ShooterConstants.timeOfFlightMap.put(4.12, 8.0);
        ShooterConstants.indexerVoltageMap.put(4.12, 12.0);

        // ShooterConstants.hoodAngleMap.put(2.198, HoodConstants.hoodAngleShootTower);
        // ShooterConstants.shooterVelocityMap.put(2.198, ShooterConstants.towerRPS);
        // // dummy
        // ShooterConstants.timeOfFlightMap.put(2.198, 7.0);

        Logger.recordOutput("Shooter/CommandRunning", false);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        Logger.recordOutput("Shooter/CommandRunning", true);
        if (autoShooting) {
            autoAngleAndVelocity();
        }

        if (runningFunnel && shooterVelocityRPS != ShooterConstants.defaultRPS) {
            if (passing) {
                funnelVoltage = FunnelConstants.passingFunnelVolts;
                indexerVoltage = FunnelConstants.passingIndexerVolts;
            } else {
                funnelVoltage = FunnelConstants.funnelFastVolts;
                // indexerVoltage = FunnelConstants.indexerFastVolts;
                indexerVoltage = Math.min(12.0, ShooterConstants.indexerVoltageMap.get(Drive.getInstance().getDistanceFromHub()));
            }
        } else {
            funnelVoltage = FunnelConstants.funnelSlowVolts;
            indexerVoltage = FunnelConstants.indexerSlowVolts;
        }

        // Checking to make sure we're still in range to continue running auto shooting
        // (Assuming we're still auto shooting of course)
        // if (!Drive.getInstance().isInShootingRange()) {
        //     autoShooting = false;
        // }

        Hood.getInstance().goToAngle(hoodAngleDeg);
        Shooter.getInstance().runVelocity(shooterVelocityRPS);
        Funnel.getInstance().runVoltageFunnel(funnelVoltage);
        Funnel.getInstance().runVoltageIndexer(indexerVoltage);
    }

    @Override
    public void end(boolean interrupted) {
        Logger.recordOutput("Shooter/CommandRunning", false);
    }

    public void autoAngleAndVelocity() {
        Pose2d pos = Drive.getInstance().getPose().exp(Drive.getInstance().getChassisSpeeds().toTwist2d(ShooterConstants.sotmPhaseShiftTimeSeconds));
        Pose2d hubPos = DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red
                ? FieldConstants.RED_HUB_POS
                : FieldConstants.BLUE_HUB_POS;
        double d = pos.getTranslation().getDistance(hubPos.getTranslation());
        // double h = ShooterConstants.hubHeightFromShooter;
        // double m = h + ShooterConstants.maxHeightFromHub;

        // double a = Math.atan((2 * m + 2 * Math.sqrt(m * (m - h))) / d);
        // Logger.recordOutput("ShooterCommand/CalculatedAngle", a);
        // a = MathUtil.clamp(a, Units.degreesToRadians(HoodConstants.minAngleDegrees + 3),
        //         Units.degreesToRadians(HoodConstants.maxAngleDegrees - 17));

        // hoodAngleDeg = Units.radiansToDegrees(a);

        // double velocityMultiplier = 1.0;
        // // 1.613 was the original "near" shot distance
        // if (d <= 2.413) {
        //     velocityMultiplier = 1.2;
        // } else if (d <= 3.7) {
        //     velocityMultiplier = 1.03;
        // } else if (d <= 3.94) {
        //     velocityMultiplier = 1.06;
        // } else if (d <= 4.17) {
        //     velocityMultiplier = 1.07;
        // } else {
        //     velocityMultiplier = 1.02;
        // }
        // Logger.recordOutput("ShooterCommand/VelocityMultiplier", velocityMultiplier);

        // double v = Math.sqrt((-4.9 * d * d) / (Math.cos(a) * Math.cos(a) * (h - d * Math.tan(a)))) * velocityMultiplier;

        // Optimize until close enough

        // double t = 0;
        // do {
        //     t = ShooterConstants.timeOfFlightMap.get(d);

        //     ChassisSpeeds speeds = Drive.getInstance().getChassisSpeeds();
        //     Pose2d newPos = pos.plus(new Transform2d(speeds.vxMetersPerSecond * t, speeds.vyMetersPerSecond * t, new Rotation2d(speeds.omegaRadiansPerSecond * t)));
        //     d = newPos.getTranslation().getDistance(hubPos.getTranslation());
        // } while (Math.abs(t - ShooterConstants.timeOfFlightMap.get(d)) > ShooterConstants.sotmOptimizationTimeThresholdSeconds);

        double v = ShooterConstants.shooterVelocityMap.get(d);
        Logger.recordOutput("Shooter/UnclampedVelocity", v);
        double a = ShooterConstants.hoodAngleMap.get(d);
        Logger.recordOutput("Shooter/UnclampedAngle", a);

        shooterVelocityRPS = MathUtil.clamp(
                v,
                ShooterConstants.minVelocityRPS,
                ShooterConstants.maxVelocityRPS);

        hoodAngleDeg = MathUtil.clamp(a, HoodConstants.minAngleDegrees, HoodConstants.maxAngleDegrees);

        Logger.recordOutput("Shooter/AutoAngle", hoodAngleDeg);
        Logger.recordOutput("Shooter/AutoVelocity", shooterVelocityRPS);
    }

    @AutoLogOutput(key = "Shooter/AutoShooting")
    public boolean isAutoShooting() {
        return autoShooting;
    }

    public void setAutoShooting(boolean autoShoot) {
        if (autoShoot) {
            autoShooting = true;
        } else {
            autoShooting = false;
        }
    }

    @AutoLogOutput(key = "Drive/AutoRotating")
    public boolean isAutoRotating() {
        return autoRotating;
    }

    public void setAutoRotating(boolean autoRotate) {
        if (autoRotate) {
            autoRotating = true;
        } else {
            autoRotating = false;
        }
    }

    // Funnel Commands
    public void runFunnel() {
        if (passing || (Shooter.getInstance().isWithinVelocityThreshold() && Hood.getInstance().isWithinAngleThreshold())) {
            runningFunnel = true;
        } else {
            runningFunnel = false;
        }
    }

    public boolean isReady() {
        return Shooter.getInstance().isWithinVelocityThreshold() && Hood.getInstance().isWithinAngleThreshold();
    }

    public void stopFunnel() {
        runningFunnel = false;
    }

    public void shootTrench() {
        setHoodAngle(HoodConstants.hoodAngleShootTrench);
        setShooterVelocity(ShooterConstants.trenchRPS);
    }

    public void shootMiddle() {
        setHoodAngle(HoodConstants.hoodAngleShootMiddle);
        setShooterVelocity(ShooterConstants.middleRPS);
    }

    public void shootTower() {
        passing = false;
        setHoodAngle(tuneHoodAngleDeg.get());
        setShooterVelocity(tuneShooterVelocityRPS.get());
    }

    public void shootNear() {
        passing = false;
        setHoodAngle(HoodConstants.hoodAngleShootNear);
        setShooterVelocity(ShooterConstants.nearRPS);
    }

    public void nearPass() {
        passing = true;
        setHoodAngle(HoodConstants.minAngleDegrees);
        setShooterVelocity(ShooterConstants.passingNeutralRPS);
        // if (Drive.getInstance().isInShootingRange()) {
        // // Can't pass
        // return;
        // } else if (Drive.getInstance().isInOpponentShootingRange()) {
        // hoodAngleDeg = HoodConstants.hoodAnglePassOpponentZone;
        // shooterVelocityMPS = ShooterConstants.passingOpponentMPS;
        // } else {
        // hoodAngleDeg = HoodConstants.hoodAnglePassNeutralZone;
        // shooterVelocityMPS = ShooterConstants.passingNeutralMPS;
        // }
    }

    public void farPass() {
        passing = true;
        setHoodAngle(HoodConstants.minAngleDegrees);
        setShooterVelocity(ShooterConstants.passingOpponentRPS);
    }

    public void reset() {
        setHoodAngle(HoodConstants.maxAngleDegrees);
        setShooterVelocity(Drive.getInstance().isInOpponentShootingRange() ? 0 : ShooterConstants.defaultRPS);
        stopFunnel();
        passing = false;
        autoShooting = false;
    }

    public void setHoodAngle(double angleDeg) {
        hoodAngleDeg = angleDeg;
    }

    public void setShooterVelocity(double VelocityMPS) {
        shooterVelocityRPS = VelocityMPS;
    }
}
