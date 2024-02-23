package frc.robot.swerve;

import java.util.List;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.UIConstants;

public class SwerveDrivetrain {
    private static AHRS gyroscope;

    private static List<SwerveModule> modules;

    private static SwerveDriveKinematics kinematics;
    private static SwerveDriveOdometry odometry;

    private static GenericEntry turnPEntry;
    private static GenericEntry turnIEntry;
    private static GenericEntry turnDEntry;
    private static GenericEntry updatePid;

    private static GenericEntry fineTune;
    public static boolean shouldFineTune;

    public static void printInformation() {
        modules.stream().forEach(m -> m.printInformation());
    }

    public static void onPeriodic() {
        modules.stream().forEach(m -> m.onPeriodic());

        if (updatePid.getBoolean(false)) {
            updatePid.setBoolean(false);
            double newP = turnPEntry.getDouble(SwerveConstants.turnP);
            double newI = turnIEntry.getDouble(SwerveConstants.turnI);
            double newD = turnDEntry.getDouble(SwerveConstants.turnD);
            new SaveableDouble("turnp", 0.0).set(newP);
            new SaveableDouble("turni", 0.0).set(newI);
            new SaveableDouble("turnd", 0.0).set(newD);
            modules.stream().forEach(m -> {
                m.updateTurningPID(newP, newI, newD);
            });
        }

        shouldFineTune = fineTune.getBoolean(shouldFineTune);
    }

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

        turnPEntry = UIConstants.tuning
            .add("Turn P Gain", SwerveConstants.turnP)
            .withSize(1, 1)
            .withPosition(3, 4)
            .getEntry();
        turnIEntry = UIConstants.tuning
            .add("Turn I Gain", SwerveConstants.turnI)
            .withSize(1, 1)
            .withPosition(4, 4)
            .getEntry();
        turnDEntry = UIConstants.tuning
            .add("Turn D Gain", SwerveConstants.turnD)
            .withSize(1, 1)
            .withPosition(5, 4)
            .getEntry();
        updatePid = UIConstants.tuning
        .add("Update PID", false)
        .withSize(1, 1)
        .withPosition(6, 4)
        .withWidget(BuiltInWidgets.kToggleButton)
        .getEntry();
        fineTune = UIConstants.tuning
        .add("Fine Tune", false)
        .withSize(1, 1)
        .withPosition(2, 4)
        .withWidget(BuiltInWidgets.kToggleButton)
        .getEntry();
    }

    public static void drive(
        double xSpeed,
        double ySpeed,
        double rot,
        boolean fieldRel,
        double periodSeconds
        ) {
        var moduleStates = kinematics.toSwerveModuleStates(
            ChassisSpeeds.discretize(ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, new Rotation2d(gyroscope.getAngle() * -1 / 180 * Math.PI)), periodSeconds
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
