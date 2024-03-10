package frc.robot.swerve;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants.UIConstants;
import frc.robot.utils.PIDSettings;

/** A class that represents a swerve turning motor. */
public class SwerveTurnMotor {
    private CANSparkMax motor;
    private SwerveEncoder encoder;

    private PIDController turningController;

    private double rotationTarget;
    private double rotationSpeed;

    /** Create a new SwerveTurningMotor
     * @param motorPort The port of the turning motor.
     * @param encoder The encoder attached to this motor.
     * @param pid The PID settings to use.
     */
    public SwerveTurnMotor(int motorPort, SwerveEncoder encoder, PIDSettings pid, boolean inverted) {
        motor = new CANSparkMax(motorPort, MotorType.kBrushless);
        this.encoder = encoder;
        turningController = pid.toController();
        turningController.enableContinuousInput(-Math.PI, Math.PI);

        motor.setInverted(inverted);

        rotationTarget = 0.0;
        rotationSpeed = 0.0;
    }

    /** Set the target angle of the module.
     * @param target The target angle.
     */
    public void setTarget(double target) {
        rotationTarget = target;
        rotationSpeed = turningController.calculate(encoder.getAngle().getRadians(), target);
        motor.set(rotationSpeed);
    }

    /** Update the PID gains of this motor.
     * @param pid The new gains to use.
     */
    public void updateGains(PIDSettings pid) {
        pid.copyTo(turningController);
    }

    /** Initialize the motor's UI on Shuffleboard.
     * @param name The name of the motor this encoder is attached to.
     * @param column The column to place UI elements on.
     */
    public void setupUI(String name, int column) {
        UIConstants.debug
            .addDouble(
                name + " target",
                () -> rotationTarget)
            .withPosition(column, 0);

        UIConstants.debug
            .addDouble(
                name + " turning",
                () -> rotationSpeed)
            .withPosition(column, 1);
    }
}
