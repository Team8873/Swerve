package frc.robot.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class Intake {
    private CANSparkMax motor;

    public Intake(int motorPort) {
        motor = new CANSparkMax(motorPort, MotorType.kBrushless);
    }

    public void setSpeed(double speed) {
        motor.set(speed);
    }
}
