// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.UIConstants;
import frc.robot.swerve.SwerveDrivetrain;
import frc.robot.utils.ParameterStore;
import frc.robot.utils.Position;
import frc.robot.utils.SimpleButton;
public class Robot extends TimedRobot {
  private final AHRS gyroscope = new AHRS();

  private final XboxController controller = new XboxController(DriveConstants.controllerPort);

  private CANSparkMax armLeft = new CANSparkMax(13, MotorType.kBrushless);
  private CANSparkMax armRight = new CANSparkMax(14, MotorType.kBrushless);

  private RelativeEncoder woahThereBuster = armLeft.getEncoder();
  private SparkAbsoluteEncoder lies = armLeft.getAbsoluteEncoder(Type.kDutyCycle);

  private CANSparkMax shooterLeft = new CANSparkMax(15, MotorType.kBrushless);
  private CANSparkMax shooterRight = new CANSparkMax(16, MotorType.kBrushless);

  private CANSparkMax succ = new CANSparkMax(17, MotorType.kBrushless);

  private SliderFella armSlida = new SliderFella(-0.24, "arm rotatey", UIConstants.arming);
  private SliderFella succSlida = new SliderFella(1.0, "the zucc", UIConstants.arming);
  private SliderFella bangBangSlida = new SliderFella(0.72, "pew pew", UIConstants.arming);

  private double $$$$ = 0.0;
  
  @Override
  public void robotInit() {
    SwerveDrivetrain.init(gyroscope);

    SimpleButton.createButton(UIConstants.tuning, "Save", new Position(0, 6), ParameterStore::saveStore);

    armRight.setInverted(true);
    armLeft.setIdleMode(IdleMode.kBrake);
    armRight.setIdleMode(IdleMode.kBrake);
  }

  @Override
  public void autonomousPeriodic() {

  }

  @Override
  public void robotPeriodic() {
    SimpleButton.updateAll();
    SmartDashboard.putNumber("angle", gyroscope.getAngle());
    SmartDashboard.putNumber("yaw", gyroscope.getYaw());
    SmartDashboard.putNumber("shmlarm", woahThereBuster.getPosition());
    SmartDashboard.putNumber("absolutely", lies.getPosition());
    if (controller.getBButton() && controller.getPOV() == 90) {
      woahThereBuster.setPosition(0);
      $$$$ = 0;
    }
    if (controller.getAButton()) {
      if (controller.getPOV() == 180) {
        $$$$ = -0.5;
      }
      if (controller.getPOV() == 0) {
        $$$$ = -30.0;
      }
      if (controller.getPOV() == 270) {
        $$$$ = -74.0;
      }
    }
  }

  private boolean wasWeSpinning$$$ = false;
  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    SwerveDrivetrain.resetHoldAngle();
    wasWeSpinning$$$ = false;
  }

  private double yeOldZucc = 0.0;
  private int $$_$$ = 0;

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    var inputs = InputPacket.readFromController(controller);
    SwerveDrivetrain.drive(inputs, getPeriod());

    double armSpinny = (controller.getLeftBumper() ? -1 : 0) + (controller.getRightBumper() ? 1 : 0.0);
    double armMul = armSlida.get();
    armSpinny *= armMul;
    double shooft = controller.getLeftTriggerAxis() * bangBangSlida.get();
    double succy = controller.getRightTriggerAxis() * succSlida.get();

    if (yeOldZucc == 0.0 && shooft > 0.0) {
      $$_$$ = 10;
    }

    yeOldZucc = shooft;

    if ($$_$$ > 0) {
      shooft = 0.0;
      succy = -1.0;
      $$_$$--;
    } else if (shooft > 0.0) {
      succy = 1.0;
    }

    if (armSpinny == 0) {
      if (!wasWeSpinning$$$) {
        wasWeSpinning$$$ = true;
        $$$$ = woahThereBuster.getPosition();
      }
      
      armSpinny = ($$$$ - woahThereBuster.getPosition()) / 5.0;
      armSpinny *= .24;
      armSpinny = MathUtil.clamp(armSpinny, -0.5, 0.5);
    } else {
      wasWeSpinning$$$ = false;
    }

    if (woahThereBuster.getPosition() > -0.2) {
      armSpinny = MathUtil.clamp(armSpinny, -1.0, 0.0);
    }

    if (woahThereBuster.getPosition() < -80.0) {
      armSpinny = MathUtil.clamp(armSpinny, 0.0, 1.0);
    }

    armLeft.set(armSpinny);
    armRight.set(armSpinny);

    shooterLeft.set(shooft);
    shooterRight.set(shooft);

    succ.set(succy);
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
