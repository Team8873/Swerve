package frc.robot.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

/** A class representing the shooter on the arm */
public class Shooter {
    public CANSparkMax leftMotor;
    public CANSparkMax rightMotor;

    /** Create a new Shooter with the given left and right motors.
     * @param leftMotorPort The left motor port of the shooter.
     * @param rightMotorPort The right motor port of the shooter.
     */
    public Shooter(int leftMotorPort, int rightMotorPort) {
        leftMotor = new CANSparkMax(leftMotorPort, MotorType.kBrushless);
        rightMotor = new CANSparkMax(rightMotorPort, MotorType.kBrushless);

        leftMotor.setIdleMode(IdleMode.kBrake);
        rightMotor.setIdleMode(IdleMode.kBrake);
    }

    /** Set the speed of the shooter's motors.
     * @param speed The speed to set the motors to.
     */
    public void setSpeed(double speed) {
        leftMotor.set(speed);
        rightMotor.set(speed);
    }
}
