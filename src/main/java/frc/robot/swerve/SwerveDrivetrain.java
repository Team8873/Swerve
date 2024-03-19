package frc.robot.swerve;

import java.util.List;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.UIConstants;
import frc.robot.input.InputPacket;
import frc.robot.ui.Position;
import frc.robot.ui.SimpleButton;
import frc.robot.ui.SimpleNumber;
import frc.robot.utils.PIDSettings;

/** A class representing a complete swerve drive system */
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

    /** Initialize the swerve drive system. This function should only be called once.
     * @param gyro The gyro for the swerve system to use.
     */
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
        turnP = new SimpleNumber(UIConstants.tuning, "Turning P", new Position(3, 4), defaultPID.p());
        turnI = new SimpleNumber(UIConstants.tuning, "Turning I", new Position(4, 4), defaultPID.i());
        turnD = new SimpleNumber(UIConstants.tuning, "Turning D", new Position(5, 4), defaultPID.d());

        SimpleButton.createButton(UIConstants.tuning, "Update PID", new Position(6, 4), () -> {
            final PIDSettings settings = new PIDSettings(turnP.get(), turnI.get(), turnD.get());
            for (var module : modules) {
                module.updateTurningPID(settings);
            }
            SwerveConstants.setTurnPID(settings);
        });
    }

    /** Force the encoders to recalculate their stored angles. This function must be called periodically or the encoders will not update their angles. */
    public static void updateEncoders() {
        for (var module : modules) {
            module.updateEncoder();
        }
    }

    /** Get the current angle of the drivetrain, in radians.
     * @return The current angle of the drivetrain, in radians.
     */
    private static double getAngle() {
        return gyroscope.getAngle() / 180 * -Math.PI;
    }

    /** Reset the hold angle to the current drivetrain angle. */
    public static void resetHoldAngle() {
        holdAngle = getAngle();
    }

    /** Set the hold angle to the specified angle.
     * @param angle The angle to hold the swerve drive at, in radians.
     */
    public static void setHoldAngle(double angle) {
        holdAngle = angle;
    }

    /** Test if the robot is at the given position with the given tolerance.
     * @param x The target x position.
     * @param y The target y position.
     * @param tolerance The amount of tolerance to check with.
     * @return Whether the robot is at the target position.
     */
    public static boolean at(Translation2d target, double tolerance) {
        Translation2d pos = odometry.getPoseMeters().getTranslation();
        boolean isAt = MathUtil.isNear(pos.getX(), target.getX(), tolerance) && MathUtil.isNear(pos.getY(), target.getY(), tolerance);
        return isAt;
    }

    /** Drive the robot towards the given position.
     * @param target The position to drive the robot to.
     * @param period The curren period in milliseconds.
     */
    public static void driveTo(Translation2d target, double period) {
        Translation2d pos = odometry.getPoseMeters().getTranslation();
        double x = pos.getX() - target.getX();
        double y = pos.getY() - target.getY();

        if (x < 0 && x < -0.03) {
            x = Math.min(-3.0, -0.1);
        } else if (x > 0.03) {
            x = Math.max(3.0, 0.1);
        }

        if (y < 0 && y < -0.03) {
            y = Math.min(-3.0, -0.1);
        } else if (y > 0.03) {
            y = Math.max(3.0, 0.1);
        }

        driveRaw(x, y, 0.0, period);
    }

    /** Drive the robot with raw speeds.
     * @param xSpeed The x speed in meters per second.
     * @param ySpeed The y speed in meters per second.
     * @param rotSpeed The rotational speed in radians per second.
     * @param periodSeconds The current period in meters per second.
     */ 
    public static void driveRaw(double xSpeed, double ySpeed, double rotSpeed, double periodSeconds) {
        if ((xSpeed != 0 || ySpeed != 0) && rotSpeed == 0) {
            rotSpeed = 0.6 * (holdAngle - getAngle()) * (Math.abs(xSpeed) + Math.abs(ySpeed)) * 2;
        }

        SmartDashboard.putNumber("drive x", xSpeed);
        SmartDashboard.putNumber("drive y", ySpeed);

        var moduleStates = kinematics.toSwerveModuleStates(
            ChassisSpeeds.discretize(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeed,
                    ySpeed,
                    rotSpeed,
                    new Rotation2d(gyroscope.getAngle() * -1 / 180 * Math.PI)),
                    periodSeconds
        ));

        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, SwerveConstants.maxSpeed * 2);
            
        final int size = modules.size();
        for (int i = 0; i < size; ++i) {
            modules.get(i).update(moduleStates[i]);
        }
    }

    /** Drive the robot with the given input.
     * @param inputs The InputPacket containing all inputs for the current period.
     * @param periodSeconds The duration of the current period, in seconds.
     */
    public static void drive(
        InputPacket inputs,
        double periodSeconds
        ) {
        double xSpeed = xSpeedLimiter.calculate(inputs.xSpeed()) * SwerveConstants.maxSpeed;
        double ySpeed = ySpeedLimiter.calculate(inputs.ySpeed()) * SwerveConstants.maxSpeed;
        double rotSpeed = rotSpeedLimiter.calculate(inputs.rotSpeed()) * SwerveConstants.maxAngularVelocity;

        double speedMod = MathUtil.interpolate(1, DriveConstants.slowModeModifier, inputs.slowMod());
        speedMod = MathUtil.interpolate(speedMod, 2, inputs.fastMod());

        xSpeed *= speedMod;
        ySpeed *= speedMod;
        rotSpeed *= speedMod;

        if (rotSpeed != 0.0 || !(xSpeed == 0.0 && ySpeed == 0.0)) {
            resetHoldAngle();
        }

        driveRaw(xSpeed, ySpeed, rotSpeed, periodSeconds);
    }

    /** Udpate the odometry of the swerve drive */
    public static void updateOdometry() {
        odometry.update(
            gyroscope.getRotation2d(),
            modules.stream().map(m -> m.getPosition()).toArray(s -> new SwerveModulePosition[s]));
        Translation2d pos = odometry.getPoseMeters().getTranslation();
        SmartDashboard.putNumber("bot x", pos.getX());
        SmartDashboard.putNumber("bot y", pos.getY());
    }

    /** Set the interal position and rotation of the robot used by the odometry system.
     * @param pose
     */
    public static void setPosition(Pose2d pose) {
        odometry.resetPosition(gyroscope.getRotation2d(), modules.stream().map(m -> m.getPosition()).toArray(s -> new SwerveModulePosition[s]), pose);
    }

    /** Get the current position of the robot as reported by the odometry system.
     * @return The position of the robot relative to the field.
     */
    public static Translation2d getPosition() {
        return odometry.getPoseMeters().getTranslation();
    }
}
