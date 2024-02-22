// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Function;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.UIConstants;
import frc.robot.swerve.SwerveDrivetrain;
public class Robot extends TimedRobot {
  private static Robot instance;
  private final AHRS gyroscope = new AHRS();

  private final XboxController controller = new XboxController(DriveConstants.controllerPort);
  private final SlewRateLimiter xLimiter = new SlewRateLimiter(DriveConstants.rateLimit);
  private final SlewRateLimiter yLimiter = new SlewRateLimiter(DriveConstants.rateLimit);
  private final SlewRateLimiter rotLimiter = new SlewRateLimiter(DriveConstants.rateLimit);

  private GenericEntry saveButton;
  private SendableChooser<Function<Double, Double>> transformChooser;

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
    instance = this;
    SwerveDrivetrain.init(gyroscope);
    saveButton = UIConstants.tuning
    .add("Save parameters", false)
    .withPosition(0, 4)
    .withSize(1, 1)
    .withWidget(BuiltInWidgets.kToggleButton)
    .getEntry();

    armRight.setInverted(true);
    

    armLeft.setIdleMode(IdleMode.kBrake);
    armRight.setIdleMode(IdleMode.kBrake);

    transformChooser = new SendableChooser<>();
    transformChooser.setDefaultOption(UIConstants.squareTransform, Robot::square);
    transformChooser.addOption(UIConstants.linearTransform, Robot::linear);
    transformChooser.addOption(UIConstants.twoStepTransform, Robot::twoStepLinear);
    transformChooser.onChange(Robot::setInputTransform);

    UIConstants.tuning
    .add("Input Transform", transformChooser)
    .withPosition(1, 4)
    .withSize(1, 1)
    .withWidget(BuiltInWidgets.kComboBoxChooser);

    addPeriodic(SwerveDrivetrain::onPeriodic, 10.0 / 1000.0, 5.0 / 1000.0);
  }

  @Override
  public void autonomousPeriodic() {

  }

  private int ticksSinceUppy = 0;

  @Override
  public void robotPeriodic() {
    SwerveDrivetrain.printInformation();
    SwerveDrivetrain.onPeriodic();
    if (saveButton.getBoolean(false)) {
      saveButton.setBoolean(false);
      SaveableDouble.saveStore();
    }
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
    if (controller.getPOV() != 0) {
      ticksSinceUppy++;
    }
  }

  private boolean wasWeSpinning$$$ = false;
  private int $$$ = 0;
  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    holdAngle = gyroscope.getRotation2d().getRadians();
    wasWeSpinning$$$ = false;
    $$$ = 0;
  }

  private double yeOldZucc = 0.0;
  private int $$_$$ = 0;

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    driveWithStick(true);

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

    //if (armSpinny + woahThereBuster.getVelocity() * 5 > 0.0) {
      //armSpinny = 0;
    //}

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

  private static Double square(Double in) {
    return in * in * Math.signum(in);
  }

  private static Double linear(Double in) {
    return in;
  }

  private static double threshold = 0.9;
  private static double preSlope = 0.6;
  private static double yOffset = threshold * preSlope;
  private static double slope = (1 - yOffset) / (1 - threshold);

  private static Double twoStepLinear(Double in) {
    double out = Math.abs(in);
    if (out < threshold) {
      out *= preSlope;
    } else {
      out = slope * (out - threshold) + yOffset;
    }
    return Math.copySign(out, in);
  }

  Function<Double, Double> inputTransform = Robot::linear;

  private static void setInputTransform(Function<Double, Double> transform) {
    instance.inputTransform = transform;
  }

  private double holdAngle = 0.0;

  private void driveWithStick(boolean fieldRel) {
    double xSpeed = -xLimiter.calculate(MathUtil.applyDeadband(controller.getLeftY(), DriveConstants.deadband));
    double ySpeed = -yLimiter.calculate(MathUtil.applyDeadband(controller.getLeftX(), DriveConstants.deadband));
    double rotSpeed = rotLimiter.calculate(MathUtil.applyDeadband(controller.getRightX(), DriveConstants.deadband));

    xSpeed = inputTransform.apply(xSpeed) * SwerveConstants.maxSpeed;
    ySpeed = inputTransform.apply(ySpeed) * SwerveConstants.maxSpeed;
    rotSpeed = inputTransform.apply(rotSpeed) * SwerveConstants.maxAngularVelocity;

    boolean eyo = controller.getYButton();

    if (eyo) {
      xSpeed *= 0.4;
      ySpeed *= 0.4;
      rotSpeed *= 0.4;
    }

    if (rotSpeed == 0.0 && (xSpeed != 0.0 || ySpeed != 0.0)) {
      rotSpeed = -0.6 * (holdAngle - gyroscope.getAngle() / 180 * Math.PI);
      rotSpeed = MathUtil.clamp(rotSpeed, -0.5, 0.5);
    } else {
      holdAngle = gyroscope.getAngle() / 180 * Math.PI;
    }

    SmartDashboard.putNumber("xSpeed", xSpeed);
    SmartDashboard.putNumber("ySpeed", ySpeed);
    SmartDashboard.putNumber("rotSpeed", rotSpeed);

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
