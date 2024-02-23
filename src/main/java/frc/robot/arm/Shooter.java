package frc.robot.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class Shooter {
    public CANSparkMax leftMotor;
    public CANSparkMax rightMotor;

    public Shooter(int leftMotorPort, int rightMotorPort) {
        leftMotor = new CANSparkMax(leftMotorPort, MotorType.kBrushless);
        rightMotor = new CANSparkMax(rightMotorPort, MotorType.kBrushless);
    }

    public void setSpeed(double speed) {
        leftMotor.set(speed);
        rightMotor.set(speed);
    }
}
