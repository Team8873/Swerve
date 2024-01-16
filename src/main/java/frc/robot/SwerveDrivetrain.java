package frc.robot;

import java.util.List;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.robot.Constants.SwerveConstants;

public class SwerveDrivetrain {
    private static AHRS gyroscope;

    private static List<SwerveModule> modules;

    private static SwerveDriveKinematics kinematics;
    private static SwerveDriveOdometry odometry;

    public static void init(AHRS gyro) {
        gyroscope = gyro;
        gyro.reset();

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
    }

    public static void drive(
        double xSpeed,
        double ySpeed,
        double rot,
        boolean fieldRel,
        double periodSeconds
        ) {
        var moduleStates = kinematics.toSwerveModuleStates(
            ChassisSpeeds.discretize(
                fieldRel
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, gyroscope.getRotation2d())
                : new ChassisSpeeds(xSpeed, ySpeed, rot),
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
