package frc.robot.swerve;

import frc.robot.Constants.SwerveConstants;
import frc.robot.utils.PIDSettings;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/** A class representing a complete swerve module */
public class SwerveModule {
    private SwerveDriveMotor drive;

    private SwerveEncoder encoder;
    private SwerveTurnMotor turn;

    /** Create a new SwerveModule with the give settings.
     * @param settings The settings to create this module with.
     */
    public SwerveModule(ModuleSettings settings) {
        drive = new SwerveDriveMotor(settings.drivePort, SwerveConstants.drivePID);
        encoder = new SwerveEncoder(settings.encoderPort, settings.name);
        turn = new SwerveTurnMotor(settings.turnPort, encoder, SwerveConstants.getTurnPID(), settings.inverted);

        drive.setupUI(settings.name, settings.columnBase);
        turn.setupUI(settings.name, settings.columnBase + 1);
        encoder.setupUI(settings.name, settings.columnBase);
    }

    public void updateEncoder() {
        encoder.updateAngle();
    }

    /** Get the position of this module.
     * @return The position of this module.
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(drive.getPosition(), encoder.getAngle());
    }

    /** Get the state of this module.
     * @return The state of this module.
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(drive.getVelocity(), encoder.getAngle());
    }

    /** Update the motor speeds of the module to match the given state.
     * @param state The target state of the module.
     */
    public void update(SwerveModuleState state) {
        final var optimizedState = SwerveModuleState.optimize(state, encoder.getAngle());

        drive.setTarget(optimizedState.speedMetersPerSecond);
        turn.setTarget(optimizedState.angle.getRadians());
    }

    /** Update the gains of the turning motor's PID controller.
     * @param settings The new settings to use.
     */
    public void updateTurningPID(PIDSettings settings) {
        turn.updateGains(settings);
    }

    /** A class representing SwerveModule configuration settings */
    public static class ModuleSettings {
        public final int drivePort;
        public final int turnPort;
        public final int encoderPort;
        public final int columnBase;
        public final String name;
        public final boolean inverted;

        /** Create a new ModuleSettings with the given parameters
         * @param drive The port of the drive motor.
         * @param turn The port of the turn motor.
         * @param encoder The port of the turning encoder.
         * @param displayColumnn The leftmost column to display this module's Shuffleboard interface on.
         * @param name The name of this module.
         */
        public ModuleSettings(int drive, int turn, int encoder, int displayColumnn, String name, boolean isInverted) {
            drivePort = drive;
            turnPort = turn;
            encoderPort = encoder;
            columnBase = displayColumnn;
            this.name = name;
            inverted = isInverted;
        }
    }

}
