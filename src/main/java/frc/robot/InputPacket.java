package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import frc.robot.Constants.DriveConstants;
import frc.robot.utils.CommandInputReader;
import frc.robot.utils.Pov;

public record InputPacket(double xSpeed, double ySpeed, double rotSpeed, double armRotSpeed, double intakeSpeed, double shooterSpeed, boolean slowMode, ArmCommand command, boolean disableArmLimits) {

    /** Create an InputPacket from the controllers inputs.
     * @param drive The main drive controller.
     * @return The InputPacket read from the controller.
     */
    public static InputPacket readFromController(XboxController drive, XboxController operator) {
        int pov = operator.getPOV();

        ArmCommand command = ArmCommand.None;

        switch (CommandInputReader.getInstance().processHat(pov)) {
        case None:
        case _2:
        case _21:
        case _23:
        case _2147:
        {
            drive.setRumble(RumbleType.kBothRumble, 0.0);
            command = commandFromHat(pov, operator);
        } break;
        case _214:
        {
            operator.setRumble(RumbleType.kBothRumble, 1.0);
            if (operator.getAButtonPressed()) command = ArmCommand.ToShoot;
            if (operator.getBButtonPressed()) command = ArmCommand.ToAmp;
        } break;
        case _236:
        {
            operator.setRumble(RumbleType.kBothRumble, 1.0);
            if (operator.getAButtonPressed()) command = ArmCommand.ToGround;
            if (operator.getBButtonPressed()) command = ArmCommand.Zero;
        } break;
        case _21478:
        {
            operator.setRumble(RumbleType.kBothRumble, 1.0);
            if (operator.getAButtonPressed()) command = ArmCommand.TrackAmp;
        } break;
        }

        return new InputPacket(
            -MathUtil.applyDeadband(drive.getLeftY(), DriveConstants.deadband),
            -MathUtil.applyDeadband(drive.getLeftX(), DriveConstants.deadband),
            MathUtil.applyDeadband(drive.getRightX(), DriveConstants.deadband),
            // (drive.getLeftBumper() ? -1.0 : 0.0) + (drive.getRightBumper() ? 1.0 : 0.0),
            MathUtil.applyDeadband(operator.getLeftY(), DriveConstants.deadband),
            MathUtil.applyDeadband(operator.getRightTriggerAxis(), DriveConstants.deadband),
            MathUtil.applyDeadband(operator.getLeftTriggerAxis(), DriveConstants.deadband),
            drive.getRightBumper(),
            command,
            operator.getXButton());
    }

    /** Convert the current POV hat input into an arm command.
     * @param pov The current POV hat input.
     * @param controller The main drive controller.
     * @return The arm command input by the driver.
     */
    private static ArmCommand commandFromHat(int pov, XboxController controller) {
        ArmCommand command = ArmCommand.None;
        switch (pov) {
        case Pov.HAT_UP:
        {
            if (controller.getAButton()) command = ArmCommand.ToShoot;
            if (controller.getYButton()) command = ArmCommand.TrackAmp;
        } break;
        case Pov.HAT_DOWN:
        {
            if (controller.getBButton()) command = ArmCommand.Zero;
            if (controller.getAButton()) command = ArmCommand.ToGround;
        } break;
        case Pov.HAT_LEFT:
        {
            if (controller.getAButton()) command = ArmCommand.ToAmp;
        } break;
        }
        return command;
    }

    /** An enum representing the possible preset arm commands */
    public static enum ArmCommand {
        None,
        Zero,
        ToGround,
        ToShoot,
        ToAmp,
        TrackAmp,
    }
}
