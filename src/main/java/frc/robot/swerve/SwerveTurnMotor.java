package frc.robot.swerve;

import java.util.function.DoubleSupplier;

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
    public SwerveTurnMotor(int motorPort, int encoderPort, PIDSettings pid) {
        motor = new CANSparkMax(motorPort, MotorType.kBrushless);
        encoder = new SwerveEncoder();
        turningController = pid.toController();

        motor.setInverted(true);

        rotationTarget = 0.0;
        rotationSpeed = 0.0;
    }

    public void setTarget(double target) {
        rotationTarget = target;
        rotationSpeed = turningController.calculate(rotationTarget, 0.0);
    }

    public void setupUI(String name, int column) {
        DoubleSupplier targetSupplier = () -> rotationTarget;
        UIConstants.debug
            .addDouble(
                name + " target",
                targetSupplier)
            .withPosition(column, 0);

        DoubleSupplier speedSupplier = () -> rotationSpeed;
        UIConstants.debug
            .addDouble(
                name + " turning",
                speedSupplier)
            .withPosition(column, 1);
    }
}
