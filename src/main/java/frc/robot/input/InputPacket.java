package frc.robot.input;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import frc.robot.Constants.DriveConstants;
import frc.robot.tracking.Tracking.TrackingState;

/** A record representing all controller inputs used by the robot */
public record InputPacket(
    double xSpeed,
    double ySpeed,
    double rotSpeed,
    double armRotSpeed,
    double intakeSpeed,
    double shooterSpeed,
    double slowMod,
    double fastMod,
    ArmCommand command,
    boolean disableArmLimits,
    TrackingState tracking,
    boolean overrideSensor,
    double climberSpeed,
    double overrideClimberSpeed,
    boolean homeClimber) {

    public static InputPacket dummy() {
        return new InputPacket(0, 0, 0, 0, 0, 0, 0, 0, ArmCommand.None, false, TrackingState.None, false, 0, 0, false);
    }

    /** Create an InputPacket from the controllers inputs.
     * @param drive The main drive controller.
     * @return The InputPacket read from the controller.
     */
    public static InputPacket readFromController(XboxController drive, XboxController operator) {
        int pov = operator.getPOV();

        ArmCommand command = ArmCommand.None;
        TrackingState tracking = TrackingState.None;

        switch (CommandInputReader.getInstance().processHat(pov)) {
        case None:
        case _2:
        case _21:
        case _23:
        {
            drive.setRumble(RumbleType.kBothRumble, 0.0);
            command = commandFromHat(pov, operator);
            tracking = trackingFromHat(pov, operator);
        } break;
        case _214:
        {
            operator.setRumble(RumbleType.kBothRumble, 1.0);
            if (operator.getAButtonPressed()) command = ArmCommand.ToShoot;
            if (operator.getBButtonPressed()) tracking = TrackingState.Amp;
        } break;
        case _236:
        {
            operator.setRumble(RumbleType.kBothRumble, 1.0);
            if (operator.getAButtonPressed()) command = ArmCommand.ToGround;
            if (operator.getBButtonPressed()) command = ArmCommand.Zero;
        } break;
        }

        return new InputPacket(
            MathUtil.applyDeadband(drive.getLeftY(), DriveConstants.deadband),
            MathUtil.applyDeadband(drive.getLeftX(), DriveConstants.deadband),
            -MathUtil.applyDeadband(drive.getRightX(), DriveConstants.deadband),
            // (drive.getLeftBumper() ? -1.0 : 0.0) + (drive.getRightBumper() ? 1.0 : 0.0),
            -MathUtil.applyDeadband(operator.getLeftY(), DriveConstants.deadband),
            MathUtil.applyDeadband(operator.getRightTriggerAxis(), DriveConstants.deadband) + (operator.getYButton() ? -1 : 0),
            MathUtil.applyDeadband(operator.getLeftTriggerAxis(), DriveConstants.deadband),
            drive.getRightTriggerAxis(),
            drive.getLeftTriggerAxis(),
            command,
            operator.getXButton(),
            tracking,
            operator.getXButton(),
            (drive.getYButton() ? 1 : 0) + (drive.getAButton() ? -1 : 0),
            (drive.getPOV() == Pov.HAT_UP ? 1 : 0) + (drive.getPOV() == Pov.HAT_DOWN ? -1 : 0),
            operator.getYButton() && operator.getPOV() == Pov.HAT_DOWN);
    }

    /** Convert the current POV hat input into an arm command.
     * @param pov The current POV hat input.
     * @param controller The controller to read additional inputs from.
     * @return The arm command input by the driver.
     */
    private static ArmCommand commandFromHat(int pov, XboxController controller) {
        ArmCommand command = ArmCommand.None;
        switch (pov) {
        case Pov.HAT_UP:
        {
            if (controller.getAButton()) command = ArmCommand.ToShoot;
        } break;
        case Pov.HAT_DOWN:
        {
            if (controller.getBButton()) command = ArmCommand.Zero;
            if (controller.getAButton()) command = ArmCommand.ToGround;
        } break;
        case Pov.HAT_RIGHT:
        {
            if (controller.getAButton()) command = ArmCommand.Up;
        } break;
        }
        return command;
    }

    /** Convert the current POV hat input into a tracking command.
     * @param pov The current POV hat input.
     * @param controller The controller to read additional inputs from.
     * @return The tracking command input by the driver.
     */
    private static TrackingState trackingFromHat(int pov, XboxController controller) {
        TrackingState state = TrackingState.None;
        switch (pov)
        {
        case Pov.HAT_LEFT:
        {
            if (controller.getAButton()) state = TrackingState.Amp;
        } break;
        }
        return state;
    }

    /** An enum representing the possible preset arm commands */
    public static enum ArmCommand {
        None,
        Zero,
        ToGround,
        ToShoot,
        Up,
    }
}
