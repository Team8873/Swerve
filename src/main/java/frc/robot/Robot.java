// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.SwerveConstants;
public class Robot extends TimedRobot {
  private final AHRS gyroscope = new AHRS();

  private final XboxController controller = new XboxController(DriveConstants.controllerPort);
  private final SlewRateLimiter xLimiter = new SlewRateLimiter(DriveConstants.rateLimit);
  private final SlewRateLimiter yLimiter = new SlewRateLimiter(DriveConstants.rateLimit);
  private final SlewRateLimiter rotLimiter = new SlewRateLimiter(DriveConstants.rateLimit);
  @Override
  public void robotInit() {
    SwerveDrivetrain.init(gyroscope);
  }

  @Override
  public void autonomousPeriodic() {
    driveWithStick(false);
    SwerveDrivetrain.updateOdometry();
  }

  @Override
  public void robotPeriodic() {}

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    driveWithStick(true);
  }

  private void driveWithStick(boolean fieldRel) {
    final double xSpeed = -xLimiter.calculate(MathUtil.applyDeadband(controller.getLeftY(), DriveConstants.deadband)) * SwerveConstants.maxSpeed;
    final double ySpeed = -yLimiter.calculate(MathUtil.applyDeadband(controller.getLeftX(), DriveConstants.deadband)) * SwerveConstants.maxSpeed;
    final double rotSpeed = -rotLimiter.calculate(MathUtil.applyDeadband(controller.getRightY(), DriveConstants.deadband)) * SwerveConstants.maxAngularVelocity;

    SwerveDrivetrain.drive(xSpeed, ySpeed, rotSpeed, fieldRel, getPeriod());
  }

  //#region literally useless
  @Override
  public void autonomousInit() {}
  @Override
  public void disabledInit() {}
  @Override
  public void disabledPeriodic() {}
  @Override
  public void testInit() {}
  @Override
  public void testPeriodic() {}
  @Override
  public void simulationInit() {}
  @Override
  public void simulationPeriodic() {}
  //#endregion
}
