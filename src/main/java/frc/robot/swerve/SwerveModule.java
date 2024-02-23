package frc.robot.swerve;

import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.UIConstants;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

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

    private SparkPIDController drivingPid;
    private ProfiledPIDController turningPid = new ProfiledPIDController(
        SwerveConstants.turnP,
        SwerveConstants.turnI,
        SwerveConstants.turnD,
        new TrapezoidProfile.Constraints(
            SwerveConstants.maxWheelAngularVelocity,
            SwerveConstants.maxWheelAngularAcceleration));

    private MedianFilter encoderFilter = new MedianFilter(SwerveConstants.filterWindow);
    private double filteredAngle;

    private GenericEntry driveSpeed;
    private GenericEntry velCurrent;
    private GenericEntry turningSpeed;
    private GenericEntry turningTarget;

    private GenericEntry angle;
    private GenericEntry rawAngle;

    private GenericEntry zeroEncoder;
    private GenericEntry turningBias;

    private GenericEntry bumpPlus;
    private GenericEntry bumpMinus;

    private GenericEntry flipEncoder;

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

        drivingPid = driveMotor.getPIDController();
        drivingPid.setP(SwerveConstants.driveP);
        drivingPid.setI(SwerveConstants.driveI);
        drivingPid.setD(SwerveConstants.driveD);
        drivingPid.setFF(1.0);

        driveEncoder = driveMotor.getEncoder();
        driveEncoder.setPositionConversionFactor(SwerveConstants.driveEncoderScaleFactor);
        driveEncoder.setVelocityConversionFactor(SwerveConstants.driveEncoderScaleFactor * 60);

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

        velCurrent = UIConstants.debug
        .addPersistent(settings.drivePort + " Velocity", 0.0)
        .withSize(1, 1)
        .withPosition(settings.columnBase + 1, 3)
        .getEntry();

        bumpPlus = UIConstants.tuning
        .add("Bump + " + settings.name, false)
        .withSize(1, 1)
        .withPosition(settings.columnBase, 2)
        .withWidget(BuiltInWidgets.kToggleButton)
        .getEntry();
        bumpMinus = UIConstants.tuning
        .add("Bump - " + settings.name, false)
        .withSize(1, 1)
        .withPosition(settings.columnBase + 1, 2)
        .withWidget(BuiltInWidgets.kToggleButton)
        .getEntry();
        flipEncoder = UIConstants.tuning
        .add("Flip " + settings.encoderPort, false)
        .withSize(1, 1)
        .withPosition(settings.columnBase + 1, 1)
        .withWidget(BuiltInWidgets.kToggleButton)
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
        else if (bumpPlus.getBoolean(false)) {
            bumpPlus.setBoolean(false);
            encoderOffset += SwerveDrivetrain.shouldFineTune ? 0.005 : 0.001;
            new SaveableDouble(settings.name, 0.0).set(encoderOffset);
        }
        else if (bumpMinus.getBoolean(false)) {
            bumpMinus.setBoolean(false);
            encoderOffset -= SwerveDrivetrain.shouldFineTune ? 0.005 : 0.001;
            new SaveableDouble(settings.name, 0.0).set(encoderOffset);
        }
        if (flipEncoder.getBoolean(false)) {
            flipEncoder.setBoolean(false);
            encoderOffset += 0.5;
            if (encoderOffset > 0.5) encoderOffset -= 1;
            new SaveableDouble(settings.name, 0.0).set(encoderOffset);
        }
        filteredAngle = encoderFilter.calculate(turnEncoder.getAbsolutePosition().getValueAsDouble());
        filteredAngle += encoderOffset;
        if (filteredAngle < -0.5) filteredAngle += 1;
        if (filteredAngle > 0.5) filteredAngle -= 1;
        filteredAngle *= SwerveConstants.turnEncoderScaleFactor;
        velCurrent.setDouble(driveEncoder.getVelocity());
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

        final double turnTarget = -turningPid.calculate(filteredAngle, optimizedState.angle.getRadians());

        turningSpeed.setDouble(turnTarget);
        turningTarget.setDouble(optimizedState.angle.getRadians());
        drivingPid.setReference(optimizedState.speedMetersPerSecond, ControlType.kVelocity);
        driveSpeed.setDouble(driveMotor.get());
        turnMotor.set(turnTarget);
    }

    public void updateDrivingPID(double p, double i, double d) {
        //drivePid.setPID(p, i, d);
    }

    public void updateTurningPID(double p, double i, double d) {
        turningPid.setPID(p, i, d);
    }
}
