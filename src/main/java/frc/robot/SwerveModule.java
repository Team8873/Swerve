package frc.robot;

import frc.robot.Constants.SwerveConstants;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class SwerveModule {
    private CANcoder turnEncoder;
    private CANSparkMax turnMotor;
    private CANSparkMax driveMotor;
    private RelativeEncoder driveEncoder;

    private double encoderOffset;

    private PIDController drivePid = new PIDController(SwerveConstants.driveP, SwerveConstants.driveI, SwerveConstants.driveD);
    private ProfiledPIDController turningPid = new ProfiledPIDController(
        SwerveConstants.turnP,
        SwerveConstants.turnI,
        SwerveConstants.turnD,
        new TrapezoidProfile.Constraints(
            SwerveConstants.maxAngularVelocity,
            SwerveConstants.maxAngularAcceleration));

    public static class ModuleSettings {
        public final int drivePort;
        public final int turnPort;
        public final int encoderPort;
        public final double offset;

        public ModuleSettings(int drive, int turn, int encoder, double offset) {
            drivePort = drive;
            turnPort = turn;
            encoderPort = encoder;
            this.offset = offset;
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
    }

    public double getAngle() {
        return turnEncoder.getAbsolutePosition().getValueAsDouble() * SwerveConstants.turnEncoderScaleFactor + encoderOffset;
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(driveMotor.getEncoder().getPosition(), new Rotation2d(getAngle()));
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(driveMotor.getEncoder().getVelocity(), new Rotation2d(getAngle()));
    }

    public void update(SwerveModuleState state) {
        final var rotation = new Rotation2d(getAngle());

        final var optimizedState = SwerveModuleState.optimize(state, rotation);

        final double driveTarget = drivePid.calculate(driveEncoder.getVelocity(), optimizedState.speedMetersPerSecond);
        final double turnTarget = turningPid.calculate(getAngle(), optimizedState.angle.getRadians());

        driveMotor.set(driveTarget);
        turnMotor.set(turnTarget);
    }
}
