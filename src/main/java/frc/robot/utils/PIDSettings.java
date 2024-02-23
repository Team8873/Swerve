package frc.robot.utils;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.PIDController;

/** A record that contains the proportional, integral, and derivative gains used by a PID controller. */
public record PIDSettings(double p, double i, double d) { 
    /** Create a PIDController with the gains specified by this PIDSettings object */
    public PIDController toController() {
        return new PIDController(p, i, d);
    }
    
    /** Get and return the SparkPIDController of the given motor with the gains specified by this
     *  PIDSettings object.
     *  
     *  This does not set the FF gain on the pid.
     *  */
    public SparkPIDController toController(CANSparkMax motor) {
        var pid = motor.getPIDController();
        pid.setP(p);
        pid.setI(i);
        pid.setD(d);
        pid.setFF(0.0);
        return pid;
    }

    public void copyTo(PIDController other) {
        other.setPID(p, i, d);
    }
}
