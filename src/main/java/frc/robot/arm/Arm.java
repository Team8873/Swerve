package frc.robot.arm;

import frc.robot.InputPacket;
import frc.robot.Constants.ArmConstants;
import frc.robot.InputPacket.ArmCommand;

public class Arm {
    private ArmMechanism rotator;
    private Shooter shooter;
    private Intake intake;

    public Arm() {
        rotator = new ArmMechanism(ArmConstants.leftRotationPort, ArmConstants.rightRotationPort);
        shooter = new Shooter(ArmConstants.leftShooterPort, ArmConstants.rightShooterPort);
        intake = new Intake(ArmConstants.intakePort);
    }

    private int shooterSpoolCounter = 0;
    private ShooterState shooterState = ShooterState.Waiting;
    public void onModeInit() {
        rotator.resetHoldAngle();
        shooterSpoolCounter = 0;
        shooterState = ShooterState.Waiting;
    }

    private void handleArmCommand(ArmCommand command) {
        switch (command) {
        case None: {} break;
        case Zero:
        {
            rotator.zero();
        } break;
        case ToGround:
        {
            rotator.setHoldAngle(0.0);
        } break;
        case ToShoot:
        {
            rotator.setHoldAngle(-30.0);
        } break;
        case ToAmp:
        {
            rotator.setHoldAngle(-74.0);
        } break;
        }
    }

    private double shooterSpeed;
    private double intakeSpeed;
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
        }
    }

    public void handleInputs(InputPacket inputs) {
        rotator.setRotationSpeed(inputs.armRotSpeed() * ArmConstants.rotSpeed);

        handleArmCommand(inputs.command());
        handleShooterSpool(inputs);

        shooter.setSpeed(shooterSpeed);
        intake.setSpeed(intakeSpeed);
    }

    private static enum ShooterState {
        Waiting,
        Spooling,
        UpToSpeed,
    }
}
