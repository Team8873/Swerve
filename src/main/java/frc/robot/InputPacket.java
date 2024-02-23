package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import frc.robot.Constants.DriveConstants;
import frc.robot.utils.CommandInputReader;
import frc.robot.utils.Pov;

public record InputPacket(double xSpeed, double ySpeed, double rotSpeed, double armRotSpeed, double intakeSpeed, double shooterSpeed, boolean slowMode, ArmCommand command) {

    /** Create an InputPacket from the controllers inputs.
     * @param controller The main drive controller.
     * @return The InputPacket read from the controller.
     */
    public static InputPacket readFromController(XboxController controller) {
        int pov = controller.getPOV();
        ArmCommand command = ArmCommand.None;

        switch (CommandInputReader.getInstance().processHat(pov)) {
            case None:
            case _2:
            case _21:
            case _23:
            {
                controller.setRumble(RumbleType.kBothRumble, 0.0);
                command = commandFromHat(pov, controller);
            } break;
            case _214:
            {
                controller.setRumble(RumbleType.kBothRumble, 1.0);
                if (controller.getAButtonPressed()) command = ArmCommand.ToShoot;
                if (controller.getBButtonPressed()) command = ArmCommand.ToAmp;
            } break;
            case _236:
            {
                controller.setRumble(RumbleType.kBothRumble, 1.0);
                if (controller.getAButtonPressed()) command = ArmCommand.ToGround;
                if (controller.getBButtonPressed()) command = ArmCommand.Zero;
            }
        }

        return new InputPacket(
            -MathUtil.applyDeadband(controller.getLeftY(), DriveConstants.deadband),
            -MathUtil.applyDeadband(controller.getLeftX(), DriveConstants.deadband),
            MathUtil.applyDeadband(controller.getRightX(), DriveConstants.deadband),
            (controller.getLeftBumper() ? -1.0 : 0.0) + (controller.getRightBumper() ? 1.0 : 0.0),
            MathUtil.applyDeadband(controller.getRightTriggerAxis(), DriveConstants.deadband),
            MathUtil.applyDeadband(controller.getLeftTriggerAxis(), DriveConstants.deadband),
            controller.getYButton(),
            command);
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
    }
}
