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

package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.DriveConstants.FieldConstants;
import frc.robot.subsystems.shooter.ShooterConstants;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class DriveCommands {
    private static final double JOYSTICK_DEADBAND = 0.1;

    private DriveCommands() {
    }

    public static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
        // Apply deadband
        double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), JOYSTICK_DEADBAND);
        Rotation2d linearDirection = (x == 0 && y == 0) ? new Rotation2d(0) : new Rotation2d(x, y);

        // Square magnitude for more precise control
        linearMagnitude = linearMagnitude * linearMagnitude;

        // Return new linear velocity
        return new Pose2d(Translation2d.kZero, linearDirection)
                .transformBy(new Transform2d(linearMagnitude, 0.0, Rotation2d.kZero))
                .getTranslation();
    }

    public static double getOmegaFromJoysticks(double driverOmega) {
        double omega = MathUtil.applyDeadband(driverOmega, JOYSTICK_DEADBAND);
        return omega * omega * Math.signum(omega);
    }

    public static ChassisSpeeds getSpeedsFromJoysticks(
            double driverX, double driverY, double driverOmega) {
        // Get linear velocity
        Translation2d linearVelocity = getLinearVelocityFromJoysticks(driverX, driverY)
                .times(DriveConstants.kMaxLinearSpeed.in(MetersPerSecond));

        // Calculate angular velocity
        double omega = getOmegaFromJoysticks(driverOmega);

        return new ChassisSpeeds(
                linearVelocity.getX(), linearVelocity.getY(), omega * DriveConstants.kMaxAngularSpeed);
    }

    /**
     * Field or robot relative drive command using two joysticks (controlling linear
     * and angular
     * velocities).
     * This is also known as joystickDrive
     */
    public static Command bigDrive(
            Drive drive,
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            DoubleSupplier omegaSupplier,
            BooleanSupplier autoDrive) {

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

        return Commands.run(() -> {

            ChassisSpeeds speeds = getSpeedsFromJoysticks(
                    xSupplier.getAsDouble(),
                    ySupplier.getAsDouble(),
                    omegaSupplier.getAsDouble());

            ChassisSpeeds robotSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    speeds,
                    DriverStation.getAlliance().isPresent()
                            && DriverStation.getAlliance().get() == Alliance.Red
                                    ? drive.getRotation().plus(new Rotation2d(Math.PI))
                                    : drive.getRotation());

            if (autoDrive.getAsBoolean()) {
                drive.autoRotate(
                        robotSpeeds,
                        new Rotation2d(rotateToHub.getAsDouble()));
            } else {
                drive.runVelocity(robotSpeeds);
            }

        }, drive);
    }
}
