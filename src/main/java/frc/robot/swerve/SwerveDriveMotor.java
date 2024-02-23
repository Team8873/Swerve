package frc.robot.swerve;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants;
import frc.robot.Constants.UIConstants;
import frc.robot.utils.PIDSettings;

public class SwerveDriveMotor {
    private CANSparkMax motor;
    private SparkPIDController velocityPid;
    private RelativeEncoder encoder;
    
    private double targetVelocity;
    private double currentSpeed;

    public SwerveDriveMotor(int motorPort, PIDSettings pid) {
        motor = new CANSparkMax(motorPort, MotorType.kBrushless);
        encoder = motor.getEncoder();
        velocityPid = pid.toController(motor);
        velocityPid.setFF(1.0);

        encoder.setPositionConversionFactor(Constants.SwerveConstants.driveEncoderScaleFactor);
        // RPS to RPM
        encoder.setVelocityConversionFactor(Constants.SwerveConstants.driveEncoderScaleFactor * 60);

        targetVelocity = 0;
        currentSpeed = 0;
    }

    public void setTarget(double velocity) {
        targetVelocity = velocity;
        
        velocityPid.setReference(targetVelocity, ControlType.kVelocity);
    }

    public double getPosition() {
        return encoder.getPosition();
    }

    public double getVelocity() {
        return encoder.getVelocity();
    }

    public void setupUI(String name, int column) {
        UIConstants.debug
            .addDouble(
                name + " speed",
                () -> currentSpeed)
            .withPosition(column, 0);
    }
}
