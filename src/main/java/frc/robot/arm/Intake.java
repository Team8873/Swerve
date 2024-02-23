package frc.robot.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

/** A class representing the intake on the arm */
public class Intake {
    private CANSparkMax motor;

    /** Create a new Intake with the given motor port.
     * @param motorPort The port of the intake motor.
     */
    public Intake(int motorPort) {
    motor = new CANSparkMax(motorPort, MotorType.kBrushless);
    }

    /** Set the speed of the intake's motor.
     * @param speed The speed to set the motor to.
     */
    public void setSpeed(double speed) {
        motor.set(speed);
    }
}
