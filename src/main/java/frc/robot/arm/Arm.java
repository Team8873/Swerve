package frc.robot.arm;

import frc.robot.tracking.Tracking;
import frc.robot.Constants.ArmConstants;
import frc.robot.input.InputPacket;
import frc.robot.input.InputPacket.ArmCommand;
import frc.robot.tracking.Tracking.TrackingState;
import frc.robot.utils.TaskRunner;
import frc.robot.utils.TaskRunner.Task;

/** A class representing the complete arm mechanism */
public class Arm {
    private ArmMechanism rotator;
    private Shooter shooter;
    private Intake intake;

    private TaskRunner<InputPacket> shooterTaskRunner;
    private TaskRunner<InputPacket> armTaskRunner;

    /** Create a new Arm */
    public Arm() {
        rotator = new ArmMechanism(ArmConstants.leftRotationPort, ArmConstants.rightRotationPort);
        shooter = new Shooter(ArmConstants.leftShooterPort, ArmConstants.rightShooterPort);
        intake = new Intake(ArmConstants.intakePort);

        shooterTaskRunner = new TaskRunner<InputPacket>().withDefault((inputs) -> {
            shooter.setSpeed(0.0);
            intake.setSpeed(inputs.intakeSpeed());
        });

        armTaskRunner = new TaskRunner<InputPacket>().withDefault((inputs) -> {
            if (inputs.armRotSpeed() == 0.0) {
                rotator.resetHoldAngle();
            }
            rotator.setRotationSpeed(inputs.armRotSpeed() * ArmConstants.rotSpeed, !inputs.disableArmLimits());
        });
    }

    /** Reset internal arm values that should be reset when the the robot is enabled in any mode */
    public void onModeInit() {
        rotator.resetHoldAngle();
        shooterTaskRunner.clear();
    }

    /** Execute the given arm command.
     * @param command The arm command to process.
     */
    private void executeCommand(ArmCommand command) {
        switch (command) {
        case None: {} break;
        case Zero:
        {
            rotator.zero();
        } break;
        case ToGround:
        {
            rotator.setHoldAngle(ArmConstants.armGround);
        } break;
        case ToShoot:
        {
            rotator.setHoldAngle(ArmConstants.armShoot);
        } break;
        }
    }

    /** Process the given inputs to manipulate the arm.
     * @param inputs The InputPacket of the current period.
     */
    public void handleInputs(InputPacket inputs) {
        if (inputs.armRotSpeed() != 0.0 || inputs.command() != ArmCommand.None) {
            Tracking.get().setState(TrackingState.None);
        }

        if (Tracking.disabled()) {
            armTaskRunner.clear();
            executeCommand(inputs.command());
        }

        if (Tracking.enabled() && !armTaskRunner.isBusy()) {
            armTaskRunner.then(new Task<InputPacket>((i) -> {
                rotator.setHoldAngle(Tracking.get().getArmAngle());
            }));
        }

        if (inputs.shooterSpeed() != 0.0 && !shooterTaskRunner.isBusy()) {
            queueShooterTask();
        } else if (inputs.shooterSpeed() == 0.0) {
            shooterTaskRunner.clear();
        }

        armTaskRunner.runOnce(inputs);
        shooterTaskRunner.runOnce(inputs);
    }

    private void queueShooterTask() {
        shooterTaskRunner.then(new Task<InputPacket>((inputs) -> {
            intake.setSpeed(-1.0);
            shooter.setSpeed(0.0);
        }, 4)).then(new Task<InputPacket>((inputs) -> {
            intake.setSpeed(0.0);
            shooter.setSpeed(1.0);
        }, 10)).then(new Task<InputPacket>((inputs) -> {
            intake.setSpeed(1.0);
            shooter.setSpeed(inputs.shooterSpeed() * ArmConstants.shooterSpeed);
        }));
    }
}
