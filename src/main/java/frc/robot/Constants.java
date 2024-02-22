package frc.robot;

import java.util.Arrays;
import java.util.List;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.swerve.SwerveModule.ModuleSettings;

public class Constants {
    public static class SwerveConstants {
        // The distance between the left and right swerve modules in meters
        public static final double trackWidth = Units.inchesToMeters(19 + 3.0 / 16.0);
        // The distance between the front and back swerve modules in meters
        public static final double driveLength = Units.inchesToMeters(19 + 3.0 / 16.0);

        //public static final double gyroXOffset = -Units.inchesToMeters(5 + 3.0 / 4.0);
        //public static final double gyroYOffset = -Units.inchesToMeters(2 + 1.0 / 8.0);

        public static final double gyroXOffset = -Units.inchesToMeters(0);
        public static final double gyroYOffset = -Units.inchesToMeters(0);

        public static final int filterWindow = 1;

        // The maximum speed of the drivetrain in meters per second
        public static final double maxSpeed = Units.feetToMeters(3.0);

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
        public static final double maxAngularVelocity = 0.5 * Math.PI;
        // The maximum angular acceleration of the drive train, in radians per second per second
        public static final double maxAngularAcceleration = 4 * Math.PI;

        public static final double maxWheelAngularVelocity = 4.0 * Math.PI;
        public static final double maxWheelAngularAcceleration = 6.0 * Math.PI;

        // A list of all swerve module settings, later used by the drivetrain to initialize the swerve modules
        public static final List<ModuleSettings> settings = Arrays.asList(new ModuleSettings[] {
            // Each module needs a drive motor port, a turning motor port, an encoder port, and an encoder offset.
            // These settings assume that only the builtin drive motor encoder is used, and an external absolute 
            // turning encoder is supplied. You may need to change this depending on your setup.
            new ModuleSettings(6, 5, 9, new SaveableDouble("Front Left", 0.17578125).get(), 9, "Front Left"),
            new ModuleSettings(8, 7, 10, new SaveableDouble("Front Right", -0.120361328125).get(), 6, "Front Right"),
            new ModuleSettings(4, 3, 11, new SaveableDouble("Back Left", -0.445068359375).get(), 3, "Back Left"),
            new ModuleSettings(2, 1, 12, new SaveableDouble("Back Right",  0.33837890625).get(), 0, "Back Right"),
        });

        // The positions of all swerve modules, left to right, front to back.
        public static final Translation2d[] positions = {
            new Translation2d(driveLength / 2.0 + gyroXOffset, trackWidth / 2.0 + gyroYOffset),
            new Translation2d(driveLength / 2.0 + gyroXOffset, -trackWidth / 2.0 + gyroYOffset),
            new Translation2d(-driveLength / 2.0 + gyroXOffset, trackWidth / 2.0 + gyroYOffset),
            new Translation2d(-driveLength / 2.0 + gyroXOffset, -trackWidth / 2.0 + gyroYOffset),
        };

        // PID Constants for driving and turning
        public static final double driveP = 0.1000;
        public static final double driveI = 0.0000;
        public static final double driveD = 0.0000;

        public static final double turnP = new SaveableDouble("turnp", .333).get();
        public static final double turnI = new SaveableDouble("turni", 0.200).get();
        public static final double turnD = new SaveableDouble("turnd", .010).get();
    }

    public static class DriveConstants {
        public static final int controllerPort = 0;
        public static final double rateLimit = 2.0;
        public static final double deadband = 0.075;
    }

    public static class UIConstants {
        public static final ShuffleboardTab debug = Shuffleboard.getTab("Debug");
        public static final ShuffleboardTab tuning = Shuffleboard.getTab("Tuning");
        public static final ShuffleboardTab arming = Shuffleboard.getTab("Arm 💪");

        public static final String squareTransform = "Quadratic";
        public static final String linearTransform = "Linear";
        public static final String twoStepTransform = "Two Step Linear";
    }
}