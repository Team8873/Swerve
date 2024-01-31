package frc.robot;

import java.util.Arrays;
import java.util.List;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.SwerveModule.ModuleSettings;

public class Constants {
    public static class SwerveConstants {
        // The distance between the left and right swerve modules in meters
        public static final double trackWidth = 0.762;
        // The distance between the front and back swerve modules in meters
        public static final double driveLength = 0.762;

        public static final int filterWindow = 1;

        // The maximum speed of the drivetrain in meters per second
        public static final double maxSpeed = 0.25;

        // The number of encoder pulses in a 360 degree wheel turn
        public static final double turnRevolutionsPerPulse = 1.0;
        // The scaling factor to convert turn encoder pulses to radians
        public static final double turnEncoderScaleFactor = Math.PI * 2 / turnRevolutionsPerPulse;

        // The diameter of the wheels in meters
        public static final double wheelDiameterMeters = 0.1016;
        // The number of encoder pulses per wheel rotation
        public static final double driveRotationsPerPulse = 4096.0;
        public static final double gearRatio = 1 / 8.14;
        // The scaling factor to convert drive encoder pulses to meters
        public static final double driveEncoderScaleFactor = wheelDiameterMeters * Math.PI * gearRatio / driveRotationsPerPulse;
        // The maximum angular velocity of the drive train, in radians per second
        public static final double maxAngularVelocity = 4 * Math.PI;
        // The maximum angular acceleration of the drive train, in radians per second per second
        public static final double maxAngularAcceleration = 4 * Math.PI;
        ;

        // A list of all swerve module settings, later used by the drivetrain to initialize the swerve modules
        public static final List<ModuleSettings> settings = Arrays.asList(new ModuleSettings[] {
            // Each module needs a drive motor port, a turning motor port, an encoder port, and an encoder offset.
            // These settings assume that only the builtin drive motor encoder is used, and an external absolute 
            // turning encoder is supplied. You may need to change this depending on your setup.
            new ModuleSettings(6, 5, 11, new SaveableDouble("Front Left", 0.17578125).get(), 9, "Front Left"),
            new ModuleSettings(8, 7, 12, new SaveableDouble("Front Right", -0.120361328125).get(), 6, "Front Right"),
            new ModuleSettings(4, 3, 10, new SaveableDouble("Back Left", -0.445068359375).get(), 3, "Back Left"),
            new ModuleSettings(2, 1, 9, new SaveableDouble("Back Right",  0.33837890625).get(), 0, "Back Right")
        });

        // The positions of all swerve modules, left to right, front to back.
        public static final Translation2d[] positions = {
            new Translation2d(driveLength / 2.0, trackWidth / 2.0),
            new Translation2d(driveLength / 2.0, -trackWidth / 2.0),
            new Translation2d(-driveLength / 2.0, trackWidth / 2.0),
            new Translation2d(-driveLength / 2.0, -trackWidth / 2.0)
        };

        // PID Constants for driving and turning
        public static final double driveP = 0.5000;
        public static final double driveI = 0.0120;
        public static final double driveD = 0.0012;

        public static final double turnP = 1.05;
        public static final double turnI = 0.50;
        public static final double turnD = 0.0095;
    }

    public static class DriveConstants {
        public static final int controllerPort = 0;
        public static final double rateLimit = 3.0;
        public static final double deadband = 0.075;
    }

    public static class UIConstants {
        public static final ShuffleboardTab debug = Shuffleboard.getTab("Debug");
        public static final ShuffleboardTab tuning = Shuffleboard.getTab("Tuning");
    }
}