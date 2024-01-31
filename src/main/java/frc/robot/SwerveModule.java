package frc.robot;

import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.UIConstants;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;

public class SwerveModule {
    private CANcoder turnEncoder;
    private CANSparkMax turnMotor;
    private CANSparkMax driveMotor;
    private RelativeEncoder driveEncoder;

    private double encoderOffset;

    private ModuleSettings settings;

    private PIDController drivePid = new PIDController(SwerveConstants.driveP, SwerveConstants.driveI, SwerveConstants.driveD);
    private ProfiledPIDController turningPid = new ProfiledPIDController(
        SwerveConstants.turnP,
        SwerveConstants.turnI,
        SwerveConstants.turnD,
        new TrapezoidProfile.Constraints(
            SwerveConstants.maxAngularVelocity,
            SwerveConstants.maxAngularAcceleration));

    private MedianFilter encoderFilter = new MedianFilter(SwerveConstants.filterWindow);
    private double filteredAngle;

    private GenericEntry driveSpeed;
    private GenericEntry turningSpeed;
    private GenericEntry turningTarget;

    private GenericEntry angle;
    private GenericEntry rawAngle;

    private GenericEntry zeroEncoder;
    private GenericEntry turningBias;

    public static class ModuleSettings {
        public final int drivePort;
        public final int turnPort;
        public final int encoderPort;
        public final double offset;
        public final int columnBase;
        public final String name;

        public ModuleSettings(int drive, int turn, int encoder, double offset, int displayColumnn, String name) {
            drivePort = drive;
            turnPort = turn;
            encoderPort = encoder;
            this.offset = offset;
            columnBase = displayColumnn;
            this.name = name;
        }
    }

    public SwerveModule(ModuleSettings settings) {
        driveMotor = new CANSparkMax(settings.drivePort, MotorType.kBrushless);
        turnMotor = new CANSparkMax(settings.turnPort, MotorType.kBrushless);
        turnEncoder = new CANcoder(settings.encoderPort);

        driveEncoder = driveMotor.getEncoder();
        driveEncoder.setPositionConversionFactor(SwerveConstants.driveEncoderScaleFactor);
        driveEncoder.setVelocityConversionFactor(SwerveConstants.driveEncoderScaleFactor);

        turningPid.enableContinuousInput(-Math.PI, Math.PI);

        encoderOffset = settings.offset;
        this.settings = settings;


        driveSpeed = UIConstants.debug
        .addPersistent(settings.drivePort + " Drive", 0.0)
        .withSize(1, 1)
        .withPosition(settings.columnBase, 0)
        .getEntry();
        turningSpeed = UIConstants.debug
        .addPersistent(settings.turnPort + " Turn", 0.0)
        .withSize(1, 1)
        .withPosition(settings.columnBase + 1, 0)
        .getEntry();
        turningTarget = UIConstants.debug
        .addPersistent(settings.turnPort + " Target", 0.0)
        .withSize(1, 1)
        .withPosition(settings.columnBase + 1, 1)
        .getEntry();

        angle = UIConstants.debug
        .addPersistent(settings.encoderPort + " Angle", 0.0)
        .withSize(1, 1)
        .withPosition(settings.columnBase + 1, 2)
        .getEntry();
        rawAngle = UIConstants.debug
        .addPersistent(settings.encoderPort + " Raw Angle", 0.0)
        .withSize(1, 1)
        .withPosition(settings.columnBase, 2)
        .getEntry();

        UIConstants.debug
        .addPersistent("Module " + settings.drivePort + settings.turnPort + settings.encoderPort, settings.name)
        .withSize(1, 1)
        .withPosition(settings.columnBase, 1);

        UIConstants.tuning
        .addPersistent("Module " + settings.drivePort + settings.turnPort + settings.encoderPort, settings.name)
        .withSize(1, 1)
        .withPosition(settings.columnBase, 1);

        zeroEncoder = UIConstants.tuning
        .add("Zero " + settings.name, false)
        .withSize(1, 1)
        .withPosition(settings.columnBase, 0)
        .withWidget(BuiltInWidgets.kToggleButton)
        .getEntry();

        turningBias = UIConstants.tuning
        .add(settings.encoderPort + " Offset", 0.0)
        .withSize(1, 1)
        .withPosition(settings.columnBase + 1, 0)
        .getEntry();
    }

    public void printInformation() {
        angle.setDouble(filteredAngle);
        rawAngle.setDouble(turnEncoder.getAbsolutePosition().getValueAsDouble());
        turningBias.setDouble(encoderOffset);
    }

    public void onPeriodic() {
        boolean doZero = zeroEncoder.getBoolean(false);
        if (doZero) {
            zeroEncoder.setBoolean(false);
            zeroEncoder();
        }
        filteredAngle = encoderFilter.calculate(turnEncoder.getAbsolutePosition().getValueAsDouble());
        filteredAngle += encoderOffset;
        if (filteredAngle < -0.5) filteredAngle += 1;
        if (filteredAngle > 0.5) filteredAngle -= 1;
        filteredAngle *= SwerveConstants.turnEncoderScaleFactor;
    }

    public void zeroEncoder() {
        double a = turnEncoder.getAbsolutePosition().getValueAsDouble();
        encoderOffset = -a;
        var off = new SaveableDouble(settings.name, encoderOffset);
        off.set(encoderOffset);
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(driveMotor.getEncoder().getPosition(), new Rotation2d(filteredAngle));
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(driveMotor.getEncoder().getVelocity(), new Rotation2d(filteredAngle));
    }

    public void update(SwerveModuleState state) {
        final var rotation = new Rotation2d(filteredAngle);

        final var optimizedState = SwerveModuleState.optimize(state, rotation);

        final double driveTarget = drivePid.calculate(driveEncoder.getVelocity(), optimizedState.speedMetersPerSecond);
        final double turnTarget = turningPid.calculate(filteredAngle, optimizedState.angle.getRadians());

        driveSpeed.setDouble(driveTarget);
        turningSpeed.setDouble(turnTarget);
        turningTarget.setDouble(optimizedState.angle.getRadians());
        driveMotor.set(driveTarget);
        turnMotor.set(turnTarget);
    }

    public void updateDrivingPID(double p, double i, double d) {
        drivePid.setPID(p, i, d);
    }

    public void updateTurningPID(double p, double i, double d) {
        turningPid.setPID(p, i, d);
    }
}
