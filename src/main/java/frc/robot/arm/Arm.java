package frc.robot.arm;

import frc.robot.InputPacket;
import frc.robot.Constants.ArmConstants;
import frc.robot.InputPacket.ArmCommand;

/** A class representing the complete arm mechanism */
public class Arm {
    private ArmMechanism rotator;
    private Shooter shooter;
    private Intake intake;

    /** Create a new Arm */
    public Arm() {
        rotator = new ArmMechanism(ArmConstants.leftRotationPort, ArmConstants.rightRotationPort);
        shooter = new Shooter(ArmConstants.leftShooterPort, ArmConstants.rightShooterPort);
        intake = new Intake(ArmConstants.intakePort);
    }

    private int shooterSpoolCounter = 0;
    private ShooterState shooterState = ShooterState.Waiting;
    private boolean isTrackingAmp = false;
    /** Reset internal arm values that should be reset when the the robot is enabled in any mode */
    public void onModeInit() {
        rotator.resetHoldAngle();
        shooterSpoolCounter = 0;
        shooterState = ShooterState.Waiting;
        isTrackingAmp = false;
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
        case ToAmp:
        {
            isTrackingAmp = true;
        } break;
        case TrackAmp:
        {
        } break;

        }

        if (isTrackingAmp) {
            rotator.setHoldAngle(ArmConstants.armAmp);
        }
    }

    private double shooterSpeed;
    private double intakeSpeed;
    /** Use the given inputs to update shooterSpeed and intakeSpeed while performing automatic shooter spooling.
     * @param inputs The InputPacket of the current period.
     */
    private void handleShooterSpool(InputPacket inputs) {
        shooterSpeed = inputs.shooterSpeed();
        intakeSpeed = inputs.intakeSpeed();

        if (shooterSpeed > 0) {
            switch (shooterState) {
            case Waiting:
            {
                intakeSpeed = -1.0;
                shooterSpeed = 0.0;
                shooterSpoolCounter++;
                if (shooterSpoolCounter == 4) {
                    shooterState = ShooterState.Spooling;
                    shooterSpoolCounter = 0;
                }
            } break;
            case Spooling:
            {
                intakeSpeed = 0.0;
                shooterSpeed = 1.0;
                shooterSpoolCounter++;
                if (shooterSpoolCounter == 10) {
                    shooterState = ShooterState.UpToSpeed;
                    shooterSpoolCounter = 0;
                }
            } break;
            case UpToSpeed:
            {
                intakeSpeed = 1.0;
                shooterSpeed *= ArmConstants.shooterSpeed;
            } break;
            }
        } else {
            shooterSpoolCounter = 0;
            intakeSpeed *= ArmConstants.intakeSpeed;
            shooterState = ShooterState.Waiting;
        }
    }

    /** Process the given inputs to manipulate the arm.
     * @param inputs The InputPacket of the current period.
     */
    public void handleInputs(InputPacket inputs) {
        rotator.setRotationSpeed(inputs.armRotSpeed() * ArmConstants.rotSpeed, !inputs.disableArmLimits());

        executeCommand(inputs.command());
        handleShooterSpool(inputs);

        shooter.setSpeed(shooterSpeed);
        intake.setSpeed(intakeSpeed);
    }

    /** An enum that represents the current state of the shooter in the automatic spooling system */
    private static enum ShooterState {
        Waiting,
        Spooling,
        UpToSpeed,
    }
}
