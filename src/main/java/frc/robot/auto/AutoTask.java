package frc.robot.auto;

import java.util.Optional;
import java.util.OptionalDouble;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.AutoConstants;
import frc.robot.swerve.SwerveDrivetrain;
import frc.robot.utils.TaskRunner.Task;

/** A class that represents a task for auton */
public class AutoTask {
    private Optional<Translation2d> swerveTarget;
    private OptionalDouble angleTarget;
    private OptionalDouble armRotationTarget;
    private OptionalDouble intakeTarget;
    private OptionalDouble shooterTarget;
    private boolean spoolingShooter;

    /** Construct a new empty auton task */
    public AutoTask() {
        swerveTarget = Optional.empty();
        angleTarget = OptionalDouble.empty();
        armRotationTarget = OptionalDouble.empty();
        intakeTarget = OptionalDouble.empty();
        shooterTarget = OptionalDouble.empty();
        spoolingShooter = false;
    }
    
    /** Make the robot move to the target position with the target angle
     * @param target The position to move to
     * @param angle The angle to turn to
     * @return This task
     */
    public AutoTask moveTo(Translation2d target, double angle) {
        swerveTarget = Optional.of(target);
        angleTarget = OptionalDouble.of(angle);
        return this;
    }

    /** Make the robot intake at a certain speed
     * @param speed The speed to intake at
     * @return This task
     */
    public AutoTask intake(double speed) {
        intakeTarget = OptionalDouble.of(speed);
        return this;
    }

    /** Shoot the shooter at a certain speed
     * @param speed The speed to shoot at
     * @return This task
     */
    public AutoTask shoot(double speed) {
        shooterTarget = OptionalDouble.of(speed);
        return this;
    }

    /** Rotate the arm to a specific angle
     * @param angle The angle to rotate to
     * @return This task
     */
    public AutoTask armTo(double angle) {
        armRotationTarget = OptionalDouble.of(angle);
        return this;
    }

    /** Wait for the shooter to spool up to full speed
     * @param speed The speel to spool up at
     * @return This task
     */
    public AutoTask spoolUpShooter(double speed) {
        shoot(speed);
        spoolingShooter = true;
        return this;
    }

    /** Convert this auto task to a task runner task
     * @return The task runner task
     */
    public Task<AutoState> toTask() {
        return new Task<AutoState>((s) -> processState(s), (s) -> isDone(s));
    }

    /** Convert this auto task to a task runner task with a fixed duration
     * @param duration How many periods to run fore
     * @return The task runner task
     */
    public Task<AutoState> toDurationTask(int duration) {
        return new Task<AutoState>((s) -> processState(s), duration);
    }

    /** Check if the task is done
     * @param state The auto state processed in the last period
     * @return Whether the task is done
     */
    private boolean isDone(AutoState state) {
        boolean done = true;

        if (swerveTarget.isPresent()) {
            done = done && state.atSwerveTarget();
        }

        if (angleTarget.isPresent()) {
            done = done && MathUtil.isNear(angleTarget.getAsDouble(), SwerveDrivetrain.getAngle(), 0.02);
        }

        if (armRotationTarget.isPresent()) {
            done = done && state.atArmTarget();
        }

        if (spoolingShooter) {
            done = done && state.isShooterSpooled();
        }

        return done;
    }

    /** Process the task and update the auto state
     * @param state The auto state to update
     */
    private void processState(AutoState state) {
        if (swerveTarget.isPresent()) {
            state.targetPosition = swerveTarget.get();
            state.targetRotation = angleTarget.getAsDouble();
        }
        
        if (armRotationTarget.isPresent()) {
            state.armRotationTarget = armRotationTarget.getAsDouble();
        }

        state.intakeSpeed = intakeTarget.orElse(0);
        state.shooterSpeed = shooterTarget.orElse(0);
    }

    public static final Task<AutoState> prepShoot = new AutoTask().armTo(AutoConstants.firstShotAngle).shoot(0.8).toTask();
    public static final Task<AutoState> shoot = new AutoTask().shoot(1.0).intake(1.0).toDurationTask(10);
    public static final Task<AutoState> toSpeaker = new AutoTask().moveTo(AutoConstants.speakerPos, 0.0).toTask();
    public static final Task<AutoState> toNote1 = new AutoTask().moveTo(AutoConstants.note1Pos, 0.0).toTask();
}
