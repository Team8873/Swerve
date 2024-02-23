package frc.robot.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.UIConstants;

public class ArmMechanism {
    private CANSparkMax leftMotor;
    private CANSparkMax rightMotor;

    private RelativeEncoder encoder;
    private double holdAngle;

    public ArmMechanism(int leftMotorPort, int rightMotorPort) {
        leftMotor = new CANSparkMax(leftMotorPort, MotorType.kBrushless);
        rightMotor = new CANSparkMax(rightMotorPort, MotorType.kBrushless);

        rightMotor.setInverted(true);

        encoder = leftMotor.getEncoder();

        leftMotor.setIdleMode(IdleMode.kBrake);
        rightMotor.setIdleMode(IdleMode.kBrake);

        UIConstants.debug
            .addDouble("Arm Angle", () -> { return encoder.getPosition(); })
            .withPosition(0, 6);

        UIConstants.debug
            .addDouble("Arm Target", () -> holdAngle)
            .withPosition(1, 6);
    }

    public void zero() {
        encoder.setPosition(0);
    }

    public void setHoldAngle(double angle) {
        holdAngle = angle;
    }

    public void resetHoldAngle() {
        holdAngle = encoder.getPosition();
    }

    private static double SOFT_STOP_MIN = -80.0;
    private static double SOFT_STOP_MAX = 0.0;

    private double applySoftStop(double speed) {
        if (encoder.getPosition() < SOFT_STOP_MIN) {
            return Math.max(speed, 0);
        }
        if (encoder.getPosition() > SOFT_STOP_MAX) {
            return Math.min(speed, 0);
        }
        return speed;
    }

    private boolean wasMoving = false;
    public void setRotationSpeed(double speed) {
        if (speed == 0.0) {
            if (!wasMoving) {
                resetHoldAngle();
                wasMoving = true;
            }

            speed = -ArmConstants.angleHoldGain * (holdAngle - encoder.getPosition()) / 5.0;
            speed = MathUtil.clamp(speed, -ArmConstants.angleHoldMaxSpeed, ArmConstants.angleHoldMaxSpeed);
        }

        speed = applySoftStop(speed);
        leftMotor.set(speed);
        rightMotor.set(speed);
    }
}
