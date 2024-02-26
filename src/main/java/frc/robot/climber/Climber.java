package frc.robot.climber;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.input.InputPacket;
import frc.robot.utils.TaskRunner;
import frc.robot.utils.TaskRunner.Task;
import frc.robot.Constants.ClimberConstants;

public class Climber {
    private CANSparkMax leftMotor;
    private CANSparkMax rightMotor;

    private RelativeEncoder leftEncoder;
    private RelativeEncoder rightEncoder;

    private TaskRunner<InputPacket> leftTaskRunner;
    private TaskRunner<InputPacket> rightTaskRunner;

    public Climber() {
        leftMotor = new CANSparkMax(ClimberConstants.leftMotorPort, MotorType.kBrushless);
        rightMotor = new CANSparkMax(ClimberConstants.rightMotorPort, MotorType.kBrushless);

        leftEncoder = leftMotor.getEncoder();
        rightEncoder = rightMotor.getEncoder();

        leftTaskRunner = new TaskRunner<InputPacket>().withDefault((input) -> {
            double speed = input.climberSpeed() * ClimberConstants.climberMaxSpeed;
            if (leftEncoder.getPosition() < ClimberConstants.climberMin) {
                speed = Math.max(0, speed);
            }
            if (leftEncoder.getPosition() > ClimberConstants.climberMax) {
                speed = Math.min(speed, 0);
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
            leftMotor.set(speed);
        });
    }

    public void handleInputs(InputPacket inputs) {
        if (inputs.climberSpeed() != 0.0) {
            leftTaskRunner.clear();
            rightTaskRunner.clear();
        }

        if (inputs.homeClimber() && !leftTaskRunner.isBusy() && !rightTaskRunner.isBusy()) {
            queueHomeTask(leftTaskRunner, leftMotor, leftEncoder);
            queueHomeTask(rightTaskRunner, rightMotor, rightEncoder);
        }

        leftTaskRunner.runOnce(inputs);
        rightTaskRunner.runOnce(inputs);
    }

    private void queueHomeTask(TaskRunner<InputPacket> runner, CANSparkMax motor, RelativeEncoder encoder) {
        runner.then(new Task<InputPacket>((input) -> {
            motor.set(-ClimberConstants.climberHomeSpeed);
        }, 5))
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
