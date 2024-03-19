package frc.robot.swerve;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.UIConstants;
import frc.robot.utils.PIDSettings;

/** A class representing a motor in a swerve drive module */
public class SwerveDriveMotor {
    private CANSparkMax motor;
    private SparkPIDController velocityPid;
    private RelativeEncoder encoder;
    
    private double targetVelocity;
    private double currentSpeed;

    /** Create a new SwerveDriveMotor on the given port with the given PID settings.
     * @param motorPort The port of the swerve motor.
     * @param pid The PID settings to use.
     */
    public SwerveDriveMotor(int motorPort, PIDSettings pid) {
        motor = new CANSparkMax(motorPort, MotorType.kBrushless);
        encoder = motor.getEncoder();
        velocityPid = pid.toController(motor);
        velocityPid.setFF(0.25);

        encoder.setPositionConversionFactor(Constants.SwerveConstants.driveEncoderScaleFactor);
        // RPS to RPM
        //encoder.setVelocityConversionFactor(SwerveConstants.driveEncoderScaleFactor / 60 * 4096);
        encoder.setVelocityConversionFactor(SwerveConstants.driveEncoderScaleFactor/60*SwerveConstants.driveRotationsPerPulse);

        targetVelocity = 0;
        currentSpeed = 0;

        UIConstants.debug.addDouble(motorPort + " target velocity", () -> targetVelocity);
        UIConstants.debug.addDouble(motorPort + " current velocity", () -> encoder.getVelocity());
        UIConstants.debug.addDouble(motorPort + " position", () -> encoder.getPosition());
        System.out.println(encoder.getCountsPerRevolution());
    }

    /** Set the target velocity of the motor.
     * @param velocity The target velocity, in meters per second.
     */
    public void setTarget(double velocity) {
        targetVelocity = velocity;
        
        velocityPid.setReference(targetVelocity, ControlType.kVelocity);
    }

    /** Get the position of the motor's encoder.
     * @return The position of the motor's encoder.
     */
    public double getPosition() {
        return encoder.getPosition();
    }

    /** Get the velocity of the motor.
     * @return The velocity of the motor.
     */
    public double getVelocity() {
        return encoder.getVelocity();
    }

    /** Initialize the motor's UI on Shuffleboard.
     * @param name The name of the module this motor is attached to.
     * @param column The column to place UI elements on.
     */
    public void setupUI(String name, int column) {
        UIConstants.debug
            .addDouble(
                name + " speed",
                () -> currentSpeed)
            .withPosition(column, 0);
    }
}
