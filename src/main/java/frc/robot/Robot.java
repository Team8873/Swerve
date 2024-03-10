// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.UIConstants;
import frc.robot.tracking.Tracking.TrackingState;
import frc.robot.tracking.Limelight;
import frc.robot.tracking.Tracking;
import frc.robot.arm.Arm;
import frc.robot.climber.Climber;
import frc.robot.input.InputPacket;
import frc.robot.swerve.SwerveDrivetrain;
import frc.robot.ui.Position;
import frc.robot.ui.SimpleButton;
import frc.robot.utils.DistanceSensor;
import frc.robot.utils.ParameterStore;
import frc.robot.utils.TaskRunner;
import frc.robot.utils.TaskRunner.Task;

public class Robot extends TimedRobot {
  private final AHRS gyroscope = new AHRS();
  private final XboxController driveController = new XboxController(DriveConstants.controllerPort);
  private final XboxController operatorController = new XboxController(DriveConstants.operatorPort);
  private Arm arm;
  private Climber climber;

  private Spark lights;

  // private DigitalInput theGuy;

  private SendableChooser<String> autoChooser;
  private static String auto = "Do Something";
  private static String drive = "Drive and score";
  private static String driveShort = "Drive no score";
  private static String noAUto = "Do nothing";

  @Override
  public void robotInit() {
    SwerveDrivetrain.init(gyroscope);
    arm = new Arm();
    climber = new Climber();

    // theGuy = new DigitalInput(1);

    SimpleButton.createButton(UIConstants.tuning, "Save", new Position(0,4), ParameterStore::saveStore);

    autoTaskRunner = new TaskRunner<Integer>().withDefault((i) -> {
      arm.intake.setSpeed(0.0);
      arm.shooter.setSpeed(0.0);
      arm.rotator.setRotationSpeed(0, true);
      climber.handleInputs(InputPacket.dummy());
      SwerveDrivetrain.driveRaw(0.0, 0.0, 0.0, getPeriod());
    });

    Limelight.camStreamSetup();
    DistanceSensor.init();

    lights = new Spark(2);
    autoChooser = new SendableChooser<>();
    autoChooser.setDefaultOption(auto, auto);
    autoChooser.addOption(drive, drive);
    autoChooser.addOption(driveShort, driveShort);
    autoChooser.addOption(noAUto, noAUto);

    SmartDashboard.putData("auto", autoChooser);
  }

  private TaskRunner<Integer> autoTaskRunner;
  @Override
  public void autonomousPeriodic() {
    autoTaskRunner.runOnce(0);
  }

  private static double lightsC = 0.01;

  private static boolean wasPov = false;
  private boolean wasC = false;
  private boolean isC = false;

  @Override
  public void robotPeriodic() {
    SimpleButton.updateAll();
    SwerveDrivetrain.updateEncoders();
    arm.updateEncoders();

    if (operatorController.getPOV() == 0 && !wasPov) {
       lightsC += 0.01;
       wasPov = true;
    } else if (operatorController.getPOV() == 180 && !wasPov) {
      wasPov = true;
      lightsC -= 0.01;
    } else if (operatorController.getPOV() == -1) {
      wasPov = false;
    }

    SmartDashboard.putNumber("dist", DistanceSensor.distance());

    // boolean man = theGuy.get();
    // if (wasC && !man) {
    //   isC = !isC;
    // }

    // wasC = man;

    // SmartDashboard.putBoolean("Tripped", isC);
    // SmartDashboard.putNumber("man2", man ? 1 : 0);
  }

  private boolean homed = false;
  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    SwerveDrivetrain.resetHoldAngle();
    Tracking.get().setState(TrackingState.None);
    arm.onModeInit();
    Limelight.camModeDriver();

    //if (/* homed && */ !DriverStation.isFMSAttached()) {
      climber.home();
      homed = true;
    //}

    lights.set(lightsC);
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    var inputs = InputPacket.readFromController(driveController, operatorController);
    if (inputs.tracking() != TrackingState.None) Tracking.get().setState(inputs.tracking());
    SwerveDrivetrain.drive(inputs, getPeriod());
    arm.handleInputs(inputs); 
    climber.handleInputs(inputs);

    if (arm.isShooterSpooled()) {
      driveController.setRumble(RumbleType.kBothRumble, 1.0);
    } else {
      driveController.setRumble(RumbleType.kBothRumble, 0.0);
    }
  }

  private int autoBackCounter = 0;
  private int autoForwardCounter = 0;

  @Override
  public void autonomousInit() {
    autoBackCounter = 0;
    autoForwardCounter = 0;
    Limelight.camModeVision();
    arm.onModeInit();
    climber.home();
    autoTaskRunner.clear();
    if (autoChooser.getSelected() == noAUto) return;
    if (autoChooser.getSelected() == driveShort) {
      autoTaskRunner.then(new Task<Integer>((i) -> {
        arm.rotator.setRotationSpeed(0, true);
        arm.intake.setSpeed(0.0);
        arm.shooter.setSpeed(0.0);
        SwerveDrivetrain.driveRaw(-0.5, 0.0, 0.0, getPeriod());
      }, 120));
      return;
    }
    if (autoChooser.getSelected() == drive) {
      autoTaskRunner.then(new Task<Integer>((i) -> {
        arm.rotator.setHoldAngle(Constants.AutoConstants.firstShotAngle);
        arm.rotator.setRotationSpeed(0, true);
        SwerveDrivetrain.driveRaw(0, 0, 0, getPeriod());
      }, () -> MathUtil.isNear(Constants.AutoConstants.firstShotAngle, arm.rotator.getAngle(), 4.0)))
      .then(new Task<Integer>((i) -> {
        arm.intake.setSpeed(-0.5);
        arm.shooter.setSpeed(0.0);
        arm.rotator.setRotationSpeed(0, true);
      }, 2))
      .then(new Task<Integer>((i) -> {
        arm.intake.setSpeed(0.0);
        arm.shooter.setSpeed(0.8);
        arm.rotator.setRotationSpeed(0, true);
      }, 40))
      .then(new Task<Integer>((i) -> {
        arm.intake.setSpeed(1.0);
        arm.shooter.setSpeed(0.7);
        arm.rotator.setRotationSpeed(0, true);
      }, 16))
      .then(new Task<Integer>((i) -> {
        arm.rotator.setHoldAngle(ArmConstants.armGround);
        arm.rotator.setRotationSpeed(0, true);
        SwerveDrivetrain.driveRaw(-0.5, 0.00, 0.0, getPeriod());
        autoBackCounter++;
      }, 50))
      .then(new Task<Integer>((i) -> {
        arm.rotator.setHoldAngle(ArmConstants.armGround);
        arm.rotator.setRotationSpeed(0, true);
        SwerveDrivetrain.driveRaw(-0.5, 0.2, 0.0, getPeriod());
        autoBackCounter++;
      }, 20))
      .then(new Task<Integer>((i) -> {
        arm.rotator.setHoldAngle(ArmConstants.armGround);
        arm.rotator.setRotationSpeed(0, true);
        SwerveDrivetrain.driveRaw(0.5, -0.2, 0.0, getPeriod());
        autoBackCounter++;
      }, 20))
      .then(new Task<Integer>((i) -> {
        arm.rotator.setHoldAngle(ArmConstants.armGround);
        arm.rotator.setRotationSpeed(0, true);
        SwerveDrivetrain.driveRaw(0.5, 0.00, 0.0, getPeriod());
        autoBackCounter++;
      }, 50))
      .then(new Task<Integer>((i) -> {
        arm.rotator.setHoldAngle(Constants.AutoConstants.firstShotAngle);
        arm.rotator.setRotationSpeed(0, true);
        SwerveDrivetrain.driveRaw(0, 0, 0, getPeriod());
      }, () -> MathUtil.isNear(Constants.AutoConstants.firstShotAngle, arm.rotator.getAngle(), 4.0)))
      .then(new Task<Integer>((i) -> {
        arm.intake.setSpeed(-0.5);
        arm.shooter.setSpeed(0.0);
        arm.rotator.setRotationSpeed(0, true);
      }, 2))
      .then(new Task<Integer>((i) -> {
        arm.intake.setSpeed(0.0);
        arm.shooter.setSpeed(0.8);
        arm.rotator.setRotationSpeed(0, true);
      }, 40))
      .then(new Task<Integer>((i) -> {
        arm.intake.setSpeed(1.0);
        arm.shooter.setSpeed(0.7);
        arm.rotator.setRotationSpeed(0, true);
      }, 16));
      return;
    }

    autoTaskRunner.then(new Task<Integer>((i) -> {
      arm.rotator.setHoldAngle(Constants.AutoConstants.firstShotAngle);
      arm.rotator.setRotationSpeed(0, true);
      SwerveDrivetrain.driveRaw(0, 0, 0, getPeriod());
    }, () -> MathUtil.isNear(Constants.AutoConstants.firstShotAngle, arm.rotator.getAngle(), 4.0)))
    .then(new Task<Integer>((i) -> {
      arm.intake.setSpeed(-0.5);
      arm.shooter.setSpeed(0.0);
      arm.rotator.setRotationSpeed(0, true);
    }, 2))
    .then(new Task<Integer>((i) -> {
      arm.intake.setSpeed(0.0);
      arm.shooter.setSpeed(0.8);
      arm.rotator.setRotationSpeed(0, true);
    }, 40))
    .then(new Task<Integer>((i) -> {
      arm.intake.setSpeed(1.0);
      arm.shooter.setSpeed(0.7);
      arm.rotator.setRotationSpeed(0, true);
    }, 16))
    .then(new Task<Integer>((i) -> {
      if (DistanceSensor.distance() > 10.0) {
        arm.intake.setSpeed(1.0);
        arm.shooter.setSpeed(-0.1);
      } else {
        arm.intake.setSpeed(0.0);
        arm.shooter.setSpeed(-0.1);
      }
      arm.rotator.setHoldAngle(ArmConstants.armGround);
      arm.rotator.setRotationSpeed(0, true);
      SwerveDrivetrain.driveRaw(-0.5, 0.00, 0.0, getPeriod());
      autoBackCounter++;
    }, () -> Limelight.getTagFieldPos().z() > 2.80))
    .then(new Task<Integer>((i) -> {
      if (DistanceSensor.distance() > 10.0) {
        arm.intake.setSpeed(1.0);
        arm.shooter.setSpeed(-0.1);
      } else {
        arm.intake.setSpeed(0.0);
        arm.shooter.setSpeed(-0.1);
      }
      arm.rotator.setHoldAngle(ArmConstants.armGround);
      arm.rotator.setRotationSpeed(0, true);
      SwerveDrivetrain.driveRaw(0.5, -0.02, 0.0, getPeriod());
      autoForwardCounter++;
    }, () -> autoForwardCounter >= autoBackCounter + 8))
    .then(new Task<Integer>((i) -> {
      arm.rotator.setHoldAngle(Constants.AutoConstants.firstShotAngle);
      arm.rotator.setRotationSpeed(0, true);
      SwerveDrivetrain.driveRaw(0.2, 0, 0, getPeriod());
    }, 8))
    .then(new Task<Integer>((i) -> {
      arm.rotator.setHoldAngle(Constants.AutoConstants.firstShotAngle);
      arm.rotator.setRotationSpeed(0, true);
      SwerveDrivetrain.driveRaw(0, 0, 0, getPeriod());
    }, () -> MathUtil.isNear(Constants.AutoConstants.firstShotAngle, arm.rotator.getAngle(), 4.0)))
    .then(new Task<Integer>((i) -> {
      arm.intake.setSpeed(-0.5);
      arm.shooter.setSpeed(0.0);
      arm.rotator.setRotationSpeed(0, true);
    }, 6))
    .then(new Task<Integer>((i) -> {
      arm.intake.setSpeed(0.0);
      arm.shooter.setSpeed(0.8);
      arm.rotator.setRotationSpeed(0, true);
    }, 40))
    .then(new Task<Integer>((i) -> {
      arm.intake.setSpeed(1.0);
      arm.shooter.setSpeed(0.7);
      arm.rotator.setRotationSpeed(0, true);
    }, 16));
    //.then(new Task<Integer>((i) -> {
      //arm.rotator.setRotationSpeed(0, true);
      //SwerveDrivetrain.driveRaw(0.0, 0.0, -(Limelight.getTagFieldPos().pitch() - 10) / 20.0, getPeriod());
      //arm.intake.setSpeed(1.0);
    //}, () -> DistanceSensor.distance() < 10.0))
    //.then(new Task<Integer>((i) -> {
      //arm.intake.setSpeed(-1.0);
      //arm.shooter.setSpeed(0.0);
      //arm.rotator.setRotationSpeed(0, true);
      //arm.rotator.setHoldAngle(AutoConstants.secondShotAngle);
      //SwerveDrivetrain.driveRaw(0.0, 0.0, -(Limelight.getTagFieldPos().pitch() - 10) / 20.0, getPeriod());
    //}, 4))
    //.then(new Task<Integer>((i) -> {
      //arm.intake.setSpeed(0.0);
      //arm.shooter.setSpeed(1.0);
      //arm.rotator.setRotationSpeed(0, true);
      //SwerveDrivetrain.driveRaw(0.0, 0.0, -(Limelight.getTagFieldPos().pitch() - 10) / 28.0, getPeriod());
    //}, 70))
    //.then(new Task<Integer>((i) -> {
      //arm.intake.setSpeed(1.0);
      //arm.shooter.setSpeed(0.9);
      //arm.rotator.setRotationSpeed(0, true);
    //}, 16));

  }
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
