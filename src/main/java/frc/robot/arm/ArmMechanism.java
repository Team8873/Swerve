package frc.robot.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.UIConstants;

/** A class that represents the arm's rotation mechanism */
public class ArmMechanism {
    private CANSparkMax leftMotor;
    private CANSparkMax rightMotor;

    private DutyCycleEncoder encoder;
    private PIDController armController;
    private double holdAngle;
    private double encoderOffset = 180;
    private double currentAngle;

    /** Create a new ArmMechanism with the given left and right motors.
     * @param leftMotorPort The port of the left motor.
     * @param rightMotorPort The port of the right motor.
     */
    public ArmMechanism(int leftMotorPort, int rightMotorPort) {
        leftMotor = new CANSparkMax(leftMotorPort, MotorType.kBrushless);
        rightMotor = new CANSparkMax(rightMotorPort, MotorType.kBrushless);

        rightMotor.setInverted(true);

        encoder = new DutyCycleEncoder(0);
        armController = new PIDController(0.04, 0, 0);
        armController.enableContinuousInput(0, 360);

        leftMotor.setIdleMode(IdleMode.kBrake);
        rightMotor.setIdleMode(IdleMode.kBrake);

        UIConstants.debug
            .addDouble("Arm Angle", () -> { return currentAngle; })
            .withPosition(0, 6);

        UIConstants.debug
            .addDouble("Arm Target", () -> holdAngle)
            .withPosition(1, 6);
    }

    /** Update the stored rotation of the arm */
    public void updateEncoder() {
        double angle = encoder.getAbsolutePosition();
        angle *= 360;
        angle += encoderOffset;
        angle = MathUtil.inputModulus(angle, 0, 360);
        currentAngle = angle;
    }

    /** Set the angle to hold the arm at.
     * 
     * @param angle The angle to hold the arm at.
     */
    public void setHoldAngle(double angle) {
        holdAngle = angle;
    }

    /** Reset the angle to hold the arm at to the arm's current angle */
    public void resetHoldAngle() {
        holdAngle = currentAngle;
    }

    private static double SOFT_STOP_MIN = 134.0;
    private static double SOFT_STOP_MAX = 224.0;

    /** Apply the soft stop to make sure the arm does not move to far
     * 
     * @param speed The speed before applying the soft stop.
     * @return The speed after applying the soft stop.
     */
    private double applySoftStop(double speed) {
        if (currentAngle < SOFT_STOP_MIN) {
            return Math.min(speed, 0);
        }
        if (currentAngle > SOFT_STOP_MAX) {
            return Math.max(speed, 0);
        }
        return speed;
    }

    /** Get the angle of the arm.
     * @return The angle of the arm in degrees.
     */
    public double getAngle() {
        return currentAngle;
    }

    private boolean wasMoving = false;
    /** Set the rotation speed of the arm. If the speed is zero, the arm will attempt to go to its current hold angle.
     * 
     * @param speed The speed to rotate the arm at.
     */
    public void setRotationSpeed(double speed, boolean useSoftLimit) {
        if (speed == 0.0) {
            if (!wasMoving) {
                resetHoldAngle();
                wasMoving = true;
            }

            speed = -armController.calculate(currentAngle, holdAngle);
            speed = MathUtil.clamp(speed, -ArmConstants.angleHoldMaxSpeed, ArmConstants.angleHoldMaxSpeed);
        } else {
            wasMoving = false;
        }

        if (useSoftLimit) {
            speed = applySoftStop(speed);
        }

        leftMotor.set(speed);
        rightMotor.set(speed);

        SmartDashboard.putNumber("arm", speed);
    }
}
