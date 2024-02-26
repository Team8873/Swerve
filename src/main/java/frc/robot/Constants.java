package frc.robot;

import java.util.Arrays;
import java.util.List;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.swerve.SwerveModule.ModuleSettings;
import frc.robot.utils.PIDSettings;
import frc.robot.utils.ParameterStore;

public class Constants {
    public static class SwerveConstants {
        /** The distance between the left and right swerve modules in meters */
        public static final double trackWidth = Units.inchesToMeters(19 + 3.0 / 16.0);
        /** The distance between the front and back swerve modules in meters */
        public static final double driveLength = Units.inchesToMeters(19 + 3.0 / 16.0);

        /** X offset of the gyro in meters */
        public static final double gyroXOffset = -Units.inchesToMeters(0);
        /** Y offset of the gyro in meters */
        public static final double gyroYOffset = -Units.inchesToMeters(0);

        /** The maximum speed of the drivetrain in meters per second */
        public static final double maxSpeed = Units.feetToMeters(3.0);

        /** The number of encoder pulses in a 360 degree wheel turn */
        public static final double turnRevolutionsPerPulse = 1.0;
        /** The scaling factor to convert turn encoder pulses to radians */
        public static final double turnEncoderScaleFactor = Math.PI * 2 / turnRevolutionsPerPulse;

        /** The diameter of the wheels in meters */
        public static final double wheelDiameterMeters = 0.1016;
        /** The number of encoder pulses per wheel rotation */
        public static final double driveRotationsPerPulse = 4096.0;
        public static final double gearRatio = 1 / 8.14;
        /** The scaling factor to convert drive encoder pulses to meters */
        public static final double driveEncoderScaleFactor = wheelDiameterMeters * Math.PI * gearRatio / driveRotationsPerPulse;
        /** The maximum angular velocity of the drive train, in radians per second */
        public static final double maxAngularVelocity = 0.5 * Math.PI;
        /** The maximum angular acceleration of the drive train, in radians per second per second */
        public static final double maxAngularAcceleration = 4 * Math.PI;

        /** The maximum angular velocity of the wheels, in radians per second */
        public static final double maxWheelAngularVelocity = 4.0 * Math.PI;
        /** The maximum angular acceleration of the wheels, in radians per second per second */
        public static final double maxWheelAngularAcceleration = 6.0 * Math.PI;

        /** A list of all swerve module settings, later used by the drivetrain to initialize the swerve modules */
        public static final List<ModuleSettings> settings = Arrays.asList(new ModuleSettings[] {
            new ModuleSettings(6, 5, 9,  9, "front-left"),
            new ModuleSettings(8, 7, 10, 6, "front-right"),
            new ModuleSettings(4, 3, 11, 3, "back-left"),
            new ModuleSettings(2, 1, 12, 0, "back-right"),
        });

        /** The positions of all swerve modules, left to right, front to back. */
        public static final Translation2d[] positions = {
            new Translation2d(driveLength / 2.0 + gyroXOffset, trackWidth / 2.0 + gyroYOffset),
            new Translation2d(driveLength / 2.0 + gyroXOffset, -trackWidth / 2.0 + gyroYOffset),
            new Translation2d(-driveLength / 2.0 + gyroXOffset, trackWidth / 2.0 + gyroYOffset),
            new Translation2d(-driveLength / 2.0 + gyroXOffset, -trackWidth / 2.0 + gyroYOffset),
        };

        /** PID Constants for driving */
        public static final PIDSettings drivePID = new PIDSettings(0.1, 0.0, 0.0);

        /** Load the turning PID constants from the dynamic parameter store */
        public static PIDSettings getTurnPID() {
            double p = ParameterStore.get("turn-p", 0.5);
            double i = ParameterStore.get("turn-i", 0.0);
            double d = ParameterStore.get("turn-d", 0.0);

            return new PIDSettings(p, i, d);
        }

        /** Save the given turning PID constants to the dynamic parameter store */
        public static void setTurnPID(PIDSettings settings) {
            ParameterStore.set("turn-p", settings.p());
            ParameterStore.set("turn-i", settings.i());
            ParameterStore.set("turn-d", settings.d());
        }
    }

    public static class ArmConstants {
        /** CAN port of the left arm rotation motor */
        public static final int leftRotationPort = 13;
        /** CAN port of the right arm rotation motor */
        public static final int rightRotationPort = 14;

        /** CAN port of the left shooter motor */
        public static final int leftShooterPort = 15;
        /** CAN port of the right shooter motor */
        public static final int rightShooterPort = 16;

        /** CAN port of the intake motor */
        public static final int intakePort = 17;

        /** Maximum speed of user-controlled arm rotation */
        public static final double rotSpeed = -0.24;
        /** Maximum speed of user-controlled shooting */
        public static final double shooterSpeed = 0.72;
        /** Maximum speed of user-controlled intaking */
        public static final double intakeSpeed = 1.0;

        /** Gain of the angle hold system */
        public static final double angleHoldGain = -0.24;
        /** The maximum speed of the angle hold system, on [0,1] */
        public static final double angleHoldMaxSpeed = 0.6;

        /** Angle of the arm when it is touching the ground */
        public static final double armGround = 0.0;
        /** Angle of the arm when it is at the arbitrary shooting position */
        public static final double armShoot = -30.0;
        /** Angle of the arm when it is at the amp scoring position */
        public static final double armAmp = -74.0;
    }

    public static class DriveConstants {
        /** The port of the main drive controller */
        public static final int controllerPort = 0;
        /** The port of the operator controller */
        public static final int operatorPort = 1;
        /** The maximum rate of change of the drive inputs */
        public static final double rateLimit = 2.0;
        /** The deadband for all analog axes */
        public static final double deadband = 0.075;

        /** The speed modifier for slow mode */
        public static final double slowModeModifier = 0.2;
    }

    public static class UIConstants {
        /** The debug display tab */
        public static final ShuffleboardTab debug = Shuffleboard.getTab("Debug");
        /** The tuning display tab */
        public static final ShuffleboardTab tuning = Shuffleboard.getTab("Tuning");
    }
}