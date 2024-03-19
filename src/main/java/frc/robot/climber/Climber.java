package frc.robot.climber;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.input.InputPacket;
import frc.robot.utils.TaskRunner;
import frc.robot.utils.TaskRunner.Task;
import frc.robot.Constants.ClimberConstants;

/** A class that represents the climbing mechanism */
public class Climber {
    private CANSparkMax leftMotor;
    private CANSparkMax rightMotor;

    private RelativeEncoder leftEncoder;
    private RelativeEncoder rightEncoder;

    private TaskRunner<InputPacket> leftTaskRunner;
    private TaskRunner<InputPacket> rightTaskRunner;

    /** Create a new climber instance with the default settings */
    public Climber() {
        leftMotor = new CANSparkMax(ClimberConstants.leftMotorPort, MotorType.kBrushless);
        rightMotor = new CANSparkMax(ClimberConstants.rightMotorPort, MotorType.kBrushless);

        leftEncoder = leftMotor.getEncoder();
        rightEncoder = rightMotor.getEncoder();

        leftMotor.setInverted(true);
        rightMotor.setInverted(true);

        leftMotor.setIdleMode(IdleMode.kBrake);
        rightMotor.setIdleMode(IdleMode.kBrake);

        leftTaskRunner = new TaskRunner<InputPacket>().withDefault((input) -> {
            double speed = input.climberSpeed() * ClimberConstants.climberMaxSpeed;
            if (leftEncoder.getPosition() < ClimberConstants.climberMin) {
                speed = Math.max(0, speed);
            }
            if (leftEncoder.getPosition() > ClimberConstants.climberMax) {
                speed = Math.min(speed, 0);
            }

            if (input.overrideClimberSpeed() != 0.0) {
                speed = input.overrideClimberSpeed() * ClimberConstants.climberMaxSpeed;
            }
            leftMotor.set(speed);
        });

        rightTaskRunner = new TaskRunner<InputPacket>().withDefault((input) -> {
            double speed = input.climberSpeed() * ClimberConstants.climberMaxSpeed;
            if (rightEncoder.getPosition() < ClimberConstants.climberMin) {
                speed = Math.max(0, speed);
            }
            if (rightEncoder.getPosition() > ClimberConstants.climberMax) {
                speed = Math.min(speed, 0);
            }
            if (input.overrideClimberSpeed() != 0.0) {
                speed = input.overrideClimberSpeed() * ClimberConstants.climberMaxSpeed;
            }
            rightMotor.set(speed);
        });
    }

    /** Move the climbers as specified by the provided input packet
     * @param inputs The input packet to process.
     */
    public void handleInputs(InputPacket inputs) {
        if (inputs.climberSpeed() != 0.0 || inputs.overrideClimberSpeed() != 0.0) {
            leftTaskRunner.clear();
            rightTaskRunner.clear();
        }

        if (inputs.homeClimber()) {
            home();
        }

        leftTaskRunner.runOnce(inputs);
        rightTaskRunner.runOnce(inputs);

        SmartDashboard.putBoolean("leftb", leftTaskRunner.isBusy());
        SmartDashboard.putBoolean("rightb", rightTaskRunner.isBusy());
        SmartDashboard.putNumber("leftc", leftMotor.getOutputCurrent());
        SmartDashboard.putNumber("rightc", rightMotor.getOutputCurrent());
    }

    /** Activate the climber auto-home sequence */
    public void home() {
        if (!leftTaskRunner.isBusy() && !rightTaskRunner.isBusy()) {
            queueHomeTask(leftTaskRunner, leftMotor, leftEncoder);
            queueHomeTask(rightTaskRunner, rightMotor, rightEncoder);
        }
    }

    /** Queue up a home task on the given components.
     * @param runner The arm's task runner.
     * @param motor The arm's motor.
     * @param encoder The arm's encoder.
     */
    private void queueHomeTask(TaskRunner<InputPacket> runner, CANSparkMax motor, RelativeEncoder encoder) {
        runner.then(new Task<InputPacket>((input) -> {
            motor.set(-ClimberConstants.climberHomeSpeed);
        }, 5))
        .then(new Task<InputPacket>((input) -> {
            motor.set(ClimberConstants.climberHomeSpeed);
        }, 4))
        .then(new Task<InputPacket>((input) -> {
            motor.set(ClimberConstants.climberHomeSpeed);
        }, () -> motor.getOutputCurrent() >= ClimberConstants.climberHomeCurrentThreshold))
        .then(new Task<InputPacket>((input) -> {
            encoder.setPosition(0.0);
            motor.set(0.0);
            runner.clear();
        }));
    }
}
