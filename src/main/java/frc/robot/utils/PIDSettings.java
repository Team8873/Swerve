package frc.robot.utils;

import edu.wpi.first.math.controller.PIDController;

/** A record that contains the proportional, integral, and derivative gains used by a PID controller. */
public record PIDSettings(double p, double i, double d) { 
    public PIDController toController() {
        return new PIDController(p, i, d);
    }
}