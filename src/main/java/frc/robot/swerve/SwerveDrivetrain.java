package frc.robot.swerve;

import java.util.List;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.robot.InputPacket;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.UIConstants;
import frc.robot.utils.Position;
import frc.robot.utils.SimpleButton;
import frc.robot.utils.SimpleNumber;
import frc.robot.utils.PIDSettings;

public class SwerveDrivetrain {
    private static AHRS gyroscope;

    private static List<SwerveModule> modules;

    private static SwerveDriveKinematics kinematics;
    private static SwerveDriveOdometry odometry;

    private static SimpleNumber turnP;
    private static SimpleNumber turnI;
    private static SimpleNumber turnD;

    private static SlewRateLimiter xSpeedLimiter = new SlewRateLimiter(DriveConstants.rateLimit);
    private static SlewRateLimiter ySpeedLimiter = new SlewRateLimiter(DriveConstants.rateLimit);
    private static SlewRateLimiter rotSpeedLimiter = new SlewRateLimiter(DriveConstants.rateLimit);

    private static double holdAngle;

    public static void init(AHRS gyro) {
        gyroscope = gyro;
        gyro.reset();
        holdAngle = 0.0;

        modules = SwerveConstants.settings
            .stream()
            .map(s -> new SwerveModule(s))
            .toList();
        
        kinematics = new SwerveDriveKinematics(
            SwerveConstants.positions[0],
            SwerveConstants.positions[1],
            SwerveConstants.positions[2],
            SwerveConstants.positions[3]);

        odometry = new SwerveDriveOdometry(
            kinematics,
            gyro.getRotation2d(),
            modules.stream().map(m -> m.getPosition()).toArray(s -> new SwerveModulePosition[s]));

        final PIDSettings defaultPID = SwerveConstants.getTurnPID();
        turnP = new SimpleNumber(UIConstants.tuning, "Turning P", new Position(3, 6), defaultPID.p());
        turnI = new SimpleNumber(UIConstants.tuning, "Turning I", new Position(4, 6), defaultPID.i());
        turnD = new SimpleNumber(UIConstants.tuning, "Turning D", new Position(5, 6), defaultPID.d());

        SimpleButton.createButton(UIConstants.tuning, "Update PID", new Position(6, 6), () -> {
            final PIDSettings settings = new PIDSettings(turnP.get(), turnI.get(), turnD.get());
            for (var module : modules) {
                module.updateTurningPID(settings);
            }
            SwerveConstants.setTurnPID(settings);
        });
    }

    private static double getAngle() {
        return gyroscope.getAngle() / 180 * -Math.PI;
    }

    public static void resetHoldAngle() {
        holdAngle = getAngle();
    }

    public static void drive(
        InputPacket inputs,
        double periodSeconds
        ) {
        double xSpeed = xSpeedLimiter.calculate(inputs.xSpeed()) * SwerveConstants.maxSpeed * inputs.getSpeedMod();
        double ySpeed = ySpeedLimiter.calculate(inputs.ySpeed()) * SwerveConstants.maxSpeed * inputs.getSpeedMod();
        double rotSpeed = rotSpeedLimiter.calculate(inputs.rotSpeed()) * SwerveConstants.maxAngularVelocity * inputs.getSpeedMod();

        if ((xSpeed != 0 || ySpeed != 0) && rotSpeed == 0) {
            rotSpeed = 0.6 * (holdAngle - getAngle());
        } else {
            resetHoldAngle();
        }

        var moduleStates = kinematics.toSwerveModuleStates(
            ChassisSpeeds.discretize(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeed,
                    ySpeed,
                    rotSpeed,
                    new Rotation2d(gyroscope.getAngle() * -1 / 180 * Math.PI)),
                    periodSeconds
        ));

        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, SwerveConstants.maxSpeed);
            
        final int size = modules.size();
        for (int i = 0; i < size; ++i) {
            modules.get(i).update(moduleStates[i]);
        }
    }

    public static void updateOdometry() {
        odometry.update(
            gyroscope.getRotation2d(),
            modules.stream().map(m -> m.getPosition()).toArray(s -> new SwerveModulePosition[s]));
    }
}
