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

    /**
     *  Construct a new SwerveTurnMotor with the specified settings.  
     * 
     *  To put its debug UI on shuffleboard, call the setupUI(String, int) function.
     * */
    public SwerveTurnMotor(int motorPort, SwerveEncoder encoder, PIDSettings pid) {
        motor = new CANSparkMax(motorPort, MotorType.kBrushless);
        this.encoder = encoder;
        turningController = pid.toController();
        turningController.enableContinuousInput(-Math.PI, Math.PI);

        motor.setInverted(true);

        rotationTarget = 0.0;
        rotationSpeed = 0.0;
    }

    public void setTarget(double target) {
        rotationTarget = target;
        rotationSpeed = turningController.calculate(rotationTarget, encoder.getAngle().getRadians());
    }

    public void updateGains(PIDSettings pid) {
        pid.copyTo(turningController);
    }

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
