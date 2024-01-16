package frc.robot;

import java.util.Arrays;
import java.util.List;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.SwerveModule.ModuleSettings;

public class Constants {
    public static class SwerveConstants {
        // The distance between the left and right swerve modules in meters
        public static final double trackWidth = 0.762;
        // The distance between the front and back swerve modules in meters
        public static final double driveLength = 0.762;

        // The maximum speed of the drivetrain in meters per second
        public static final double maxSpeed = 1.0;

        // The number of encoder pulses in a 360 degree wheel turn
        public static final double turnRevolutionsPerPulse = 1.0;
        // The scaling factor to convert turn encoder pulses to radians
        public static final double turnEncoderScaleFactor = Math.PI * 2 / turnRevolutionsPerPulse;

        // The diameter of the wheels in meters per second
        public static final double wheelDiameterMeters = 0.0508;
        // The number of encoder pulses per wheel rotation
        public static final double driveRotationsPerPulse = 1.0;
        // The scaling factor to convert drive encoder pulses to meters
        public static final double driveEncoderScaleFactor = wheelDiameterMeters * Math.PI * 2 / driveRotationsPerPulse;

        // The maximum angular velocity of the drive train, in radians per second
        public static final double maxAngularVelocity = 2.0;
        // The maximum angular acceleration of the drive train, in radians per second per second
        public static final double maxAngularAcceleration = 6.28;

        // A list of all swerve module settings, later used by the drivetrain to initialize the swerve modules
        public static final List<ModuleSettings> settings = Arrays.asList(new ModuleSettings[] {
            // Each module needs a drive motor port, a turning motor port, an encoder port, and an encoder offset.
            // These settings assume that only the builtin drive motor encoder is used, and an external absolute 
            // turning encoder is supplied. You may need to change this depending on your setup.
            new ModuleSettings(0, 1, 2, 0.0),
            new ModuleSettings(3, 4, 5, 0.0),
            new ModuleSettings(6, 7, 8, 0.0),
            new ModuleSettings(9, 10, 11, 0.0)
        });

        // The positions of all swerve modules, left to right, front to back.
        public static final Translation2d[] positions = {
            new Translation2d(driveLength / 2.0, trackWidth / 2.0),
            new Translation2d(driveLength / 2.0, -trackWidth / 2.0),
            new Translation2d(-driveLength / 2.0, trackWidth / 2.0),
            new Translation2d(-driveLength / 2.0, -trackWidth / 2.0)
        };

        // PID Constants for driving and turning
        public static final double driveP = 1.0000;
        public static final double driveI = 0.0000;
        public static final double driveD = 0.0000;

        public static final double turnP = 1.0000;
        public static final double turnI = 0.0000;
        public static final double turnD = 0.0000;
    }

    public static class DriveConstants {
        public static final int controllerPort = 0;
        public static final double rateLimit = 3.0;
        public static final double deadband = 0.02;
    }
}
