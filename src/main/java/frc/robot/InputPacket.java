package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.DriveConstants;

public record InputPacket(double xSpeed, double ySpeed, double rotSpeed, double armRotSpeed, double intakeSpeed, double shooterSpeed, boolean slowMode, ArmCommand command) {
    private static final int HAT_UP = 0;
    @SuppressWarnings("unused")
    private static final int HAT_UP_RIGHT = 45;
    @SuppressWarnings("unused")
    private static final int HAT_RIGHT = 90;
    @SuppressWarnings("unused")
    private static final int HAT_DOWN_RIGHT = 135;
    private static final int HAT_DOWN = 180;
    @SuppressWarnings("unused")
    private static final int HAT_DOWN_LEFT = 225;
    private static final int HAT_LEFT = 270;
    @SuppressWarnings("unused")
    private static final int HAT_UP_LEFT = 315;

    public static InputPacket readFromController(XboxController controller) {
        int hat = controller.getPOV();
        ArmCommand command = ArmCommand.None;

        switch (hat) {
        case HAT_UP:
        {
            if (controller.getBButton()) command = ArmCommand.ReverseIntake;
            if (controller.getAButton()) command = ArmCommand.ToShoot;
        } break;
        case HAT_DOWN:
        {
            if (controller.getBButton()) command = ArmCommand.Zero;
            if (controller.getAButton()) command = ArmCommand.ToGround;
        } break;
        case HAT_LEFT:
        {
            if (controller.getAButton()) command = ArmCommand.ToAmp;
        } break;
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

    public double getSpeedMod() {
        return slowMode ? 0.2 : 1.0;
    }

    public static enum ArmCommand {
        None,
        Zero,
        ToGround,
        ToShoot,
        ToAmp,
        ReverseIntake,
    }
}
