// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.UIConstants;
import frc.robot.Tracking.TrackingState;
import frc.robot.arm.Arm;
import frc.robot.swerve.SwerveDrivetrain;
import frc.robot.utils.ParameterStore;
import frc.robot.utils.Position;
import frc.robot.utils.SimpleButton;

public class Robot extends TimedRobot {
  private final AHRS gyroscope = new AHRS();
  private final XboxController driveController = new XboxController(DriveConstants.controllerPort);
  private final XboxController operatorController = new XboxController(DriveConstants.operatorPort);
  private Arm arm;

  @Override
  public void robotInit() {
    SwerveDrivetrain.init(gyroscope);
    arm = new Arm();

    SimpleButton.createButton(UIConstants.tuning, "Save", new Position(0,4), ParameterStore::saveStore);
  }

  @Override
  public void autonomousPeriodic() { }

  @Override
  public void robotPeriodic() {
    SimpleButton.updateAll();
    SwerveDrivetrain.updateEncoders();
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    SwerveDrivetrain.resetHoldAngle();
    Tracking.getInstance().setState(TrackingState.None);
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    var inputs = InputPacket.readFromController(driveController, operatorController);
    if (inputs.tracking() != TrackingState.None) Tracking.getInstance().setState(inputs.tracking());
    SwerveDrivetrain.drive(inputs, getPeriod());
    arm.handleInputs(inputs); 
  }

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
}
