// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.UIConstants;
import frc.robot.tracking.Tracking.TrackingState;
import frc.robot.tracking.Limelight;
import frc.robot.tracking.Tracking;
import frc.robot.arm.Arm;
import frc.robot.auto.AutoState;
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
    private final Arm arm = new Arm();
    private final Climber climber = new Climber();

    private SendableChooser<String> autoChooser;
    private static String auto = "Do Something";
    private static String drive = "Drive and score";
    private static String driveShort = "Drive no score";
    private static String noAuto = "Do nothing";

    private TaskRunner<AutoState> autoTaskRunner;

    @Override
    public void robotInit() {
        SwerveDrivetrain.init(gyroscope);

        SimpleButton.createButton(UIConstants.tuning, "Save", new Position(0,4), ParameterStore::saveStore);

        Limelight.camStreamSetup();
        DistanceSensor.init();


        autoChooser = new SendableChooser<>();
        autoChooser.setDefaultOption(auto, auto);
        autoChooser.addOption(drive, drive);
        autoChooser.addOption(driveShort, driveShort);
        autoChooser.addOption(noAuto, noAuto);

        SmartDashboard.putData("auto", autoChooser);

        autoTaskRunner = new TaskRunner<>();

        autoTaskRunner.withDefault((s) -> {
            s.intakeSpeed = 0.0;
            s.shooterSpeed = 0.0;
            s.targetPosition = SwerveDrivetrain.getPosition();
        });
    }

    @Override
    public void robotPeriodic() {
        SimpleButton.updateAll();
        SwerveDrivetrain.updateEncoders();
        arm.updateEncoders();
        SwerveDrivetrain.updateOdometry();

        SmartDashboard.putNumber("dist", DistanceSensor.distance());
    }

    private AutoState autoState;

    @Override
    public void autonomousInit() {
        Limelight.camModeVision();
        SwerveDrivetrain.setPosition(Limelight.getRobotPos());
        arm.onModeInit();
        autoState = new AutoState(() -> arm.getArmAngle());
        autoTaskRunner
            .then(new Task<AutoState>((AutoState s) -> {
                s.armRotationTarget = AutoConstants.firstShotAngle;
                s.shooterSpeed = 0.8;
            }, () -> autoState.atArmTarget()))
            .then(new Task<AutoState>((AutoState s) -> {
                s.shooterSpeed = 1.0;
                s.intakeSpeed = 1.0;
            }, 10))
            .then(new Task<AutoState>((AutoState s) -> {
                s.shooterSpeed = 0.0;
                s.intakeSpeed = 0.0;
                s.targetPosition = new Translation2d(3.3, 7.0);
            }, () -> autoState.atSwerveTarget()))
            .then(new Task<AutoState>((AutoState s) -> {
                s.targetPosition = new Translation2d(1.56, 5.33);
            }, () -> autoState.atSwerveTarget()))
            .then(new Task<AutoState>((AutoState s) -> {
                s.done = true;
            }));
        return;
    }

    @Override
    public void autonomousPeriodic() {
        autoTaskRunner.runOnce(autoState);
        arm.setArmTargetAngle(autoState.armRotationTarget);
        arm.handleRawInputs(0.0, autoState.intakeSpeed, autoState.shooterSpeed, true);
        SwerveDrivetrain.setHoldAngle(autoState.targetRotation);
        SwerveDrivetrain.driveTo(autoState.targetPosition, getPeriod());
    }

    /** This function is called once when teleop is enabled. */
    @Override
    public void teleopInit() {
        SwerveDrivetrain.resetHoldAngle();
        Tracking.get().setState(TrackingState.None);
        arm.onModeInit();
        Limelight.camModeDriver();

        climber.home();
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
