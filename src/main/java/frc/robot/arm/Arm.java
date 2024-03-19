package frc.robot.arm;

import frc.robot.tracking.Tracking;
import frc.robot.Constants.ArmConstants;
import frc.robot.input.InputPacket;
import frc.robot.input.InputPacket.ArmCommand;
import frc.robot.tracking.Tracking.TrackingState;
import frc.robot.utils.TaskRunner;
import frc.robot.utils.TaskRunner.Task;
import frc.robot.utils.DistanceSensor;

/** A class representing the complete arm mechanism */
public class Arm {
    public ArmMechanism rotator;
    public Shooter shooter;
    public Intake intake;

    private TaskRunner<InputPacket> shooterTaskRunner;
    private TaskRunner<InputPacket> armTaskRunner;

    private boolean wasRotating = false;
    private boolean doingIntakeTask = false;

    /** Create a new Arm */
    public Arm() {
        rotator = new ArmMechanism(ArmConstants.leftRotationPort, ArmConstants.rightRotationPort);
        shooter = new Shooter(ArmConstants.leftShooterPort, ArmConstants.rightShooterPort);
        intake = new Intake(ArmConstants.intakePort);

        shooterTaskRunner = new TaskRunner<InputPacket>().withDefault((inputs) -> {
            shooter.setSpeed(inputs.shooterSpeed());
            intake.setSpeed(inputs.intakeSpeed());
        });

        armTaskRunner = new TaskRunner<InputPacket>().withDefault((inputs) -> {
            if (inputs.armRotSpeed() == 0.0) {
                if (!wasRotating) {
                    rotator.resetHoldAngle(); 
                    wasRotating = true;
                }
            } else {
                wasRotating = false;
            }
            rotator.setRotationSpeed(inputs.armRotSpeed() * ArmConstants.rotSpeed, !inputs.disableArmLimits());
        });
    }

    /** Update the stored encoder angle of the arm */
    public void updateEncoders() {
        rotator.updateEncoder();
    }
    
    /** Get the current angle of the arm, in degrees */
    public double getArmAngle() {
        return rotator.getAngle();
    }

    /** Reset internal arm values that should be reset when the the robot is enabled in any mode */
    public void onModeInit() {
        rotator.resetHoldAngle();
        shooterTaskRunner.clear();
        wasRotating = false;
        doingIntakeTask = false;
    }

    /** Execute the given arm command.
     * @param command The arm command to process.
     */
    private void executeCommand(ArmCommand command) {
        switch (command) {
        case None:
        case Zero: {} break;
        case ToGround:
        {
            rotator.setHoldAngle(ArmConstants.armGround);
        } break;
        case ToShoot:
        {
            rotator.setHoldAngle(ArmConstants.armShoot);
        } break;
        case Up:
        {
            rotator.setHoldAngle(ArmConstants.armUp);
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
                rotator.setRotationSpeed(0.0, true);
            }));
        }

        if (inputs.shooterSpeed() != 0.0) {
            if (doingIntakeTask) {
                shooterTaskRunner.clear();
                doingIntakeTask = false;
            }
        } else if (inputs.intakeSpeed() > 0.0 && !inputs.overrideSensor()) {
            doingIntakeTask = true;
            if (!shooterTaskRunner.isBusy()) {
                shooterTaskRunner.then(new Task<InputPacket>((i) -> {
                    shooter.setSpeed(-0.1);
                    intake.setSpeed(i.intakeSpeed());
                }, () -> DistanceSensor.isDetecting() && DistanceSensor.distance() < 10.0))
                .then(new Task<InputPacket>((i) -> {
                    shooter.setSpeed(0.0);
                    intake.setSpeed(-0.5);
                }, () -> DistanceSensor.distance() > 10.0)) // <--- Intake suck it back
                .then(new Task<InputPacket>((i) -> {
                    shooter.setSpeed(0.0);
                    intake.setSpeed(0.5);
                }, ArmConstants.armPushBackAfterItPullsInWhenItIntakes)) // <--- Intake push back in after it sucks it back
                .then(new Task<InputPacket>((i) -> {
                    shooter.setSpeed(0.0);
                    intake.setSpeed(Math.min(i.intakeSpeed(), 0.0));
                }));
            }
        } else if (inputs.intakeSpeed() < 0.0 || !DistanceSensor.isDetecting() || DistanceSensor.distance() > 10.0 || inputs.overrideSensor()) {
            doingIntakeTask = false;
            shooterTaskRunner.clear();
        } else {
            if (!doingIntakeTask) {
                shooterTaskRunner.clear();
            }
        }

        armTaskRunner.runOnce(inputs);
        shooterTaskRunner.runOnce(inputs);
    }

    /** Set the rotation target angle of the arm.
     * @param angle The angle to go to in degrees.
     */
    public void setArmTargetAngle(double angle) {

    }
    
    /** Run the arm using raw speed inputs.
     * @param rotation The rotation speed.
     * @param intake The intake speed.
     * @param shooter The shooter speed.
     * @param useSoftLimits Whether to use the arm soft limits.
     */
    public void handleRawInputs(double rotation, double intake, double shooter, boolean useSoftLimits) {
        rotator.setRotationSpeed(rotation, useSoftLimits);
        this.intake.setSpeed(intake);
        this.shooter.setSpeed(shooter);
    }

    /** Check if the shooter is spooled up to max velocity.
     * @return Whether the shooter is currently spooled up.
     */
    public boolean isShooterSpooled() {
        return getShooterSpeed() > ArmConstants.shooterRevSpeed;
    }

    /** Get the current speed of the shooter in rpm.
     * @return The speed of the shooter.
     */
    public double getShooterSpeed() {
        return shooter.leftMotor.getEncoder().getVelocity();
    }
}
