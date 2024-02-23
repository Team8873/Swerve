package frc.robot.swerve;

import frc.robot.Constants.SwerveConstants;
import frc.robot.utils.PIDSettings;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModule {
    private SwerveDriveMotor drive;

    private SwerveEncoder encoder;
    private SwerveTurnMotor turn;

    public SwerveModule(ModuleSettings settings) {
        drive = new SwerveDriveMotor(settings.drivePort, SwerveConstants.drivePID);
        encoder = new SwerveEncoder(settings.encoderPort, settings.name);
        turn = new SwerveTurnMotor(settings.turnPort, encoder, SwerveConstants.getTurnPID());

        drive.setupUI(settings.name, settings.columnBase);
        turn.setupUI(settings.name, settings.columnBase + 1);
        encoder.setupUI(settings.name, settings.columnBase);
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(drive.getPosition(), encoder.getAngle());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(drive.getVelocity(), encoder.getAngle());
    }

    public void update(SwerveModuleState state) {
        final var optimizedState = SwerveModuleState.optimize(state, encoder.getAngle());

        drive.setTarget(optimizedState.speedMetersPerSecond);
        turn.setTarget(optimizedState.angle.getRadians());
    }

    public void updateTurningPID(PIDSettings settings) {
        turn.updateGains(settings);
    }

    public static class ModuleSettings {
        public final int drivePort;
        public final int turnPort;
        public final int encoderPort;
        public final int columnBase;
        public final String name;

        public ModuleSettings(int drive, int turn, int encoder, int displayColumnn, String name) {
            drivePort = drive;
            turnPort = turn;
            encoderPort = encoder;
            columnBase = displayColumnn;
            this.name = name;
        }
    }

}
