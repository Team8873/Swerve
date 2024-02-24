package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.ArmConstants;
import frc.robot.utils.Limelight;

public class Tracking {
    private static Tracking instance;

    private TrackingState state = TrackingState.None;
   
    public static Tracking getInstance() {
        if (instance == null) {
            instance = new Tracking();
        }
        return instance;
    }

    public void setState(TrackingState state) {
        this.state = state;

        if (this.state == TrackingState.Amp) {
            var alliance = DriverStation.getAlliance();
            if (alliance.isEmpty()) {
                Limelight.setPriority(13);
            } else if (alliance.get() == Alliance.Red) {
                Limelight.setPriority(6);
            } else {
                Limelight.setPriority(5);
            }
        }
    }

    public TrackingState getState() {
        return state;
    }


    public double getArmAngle() {
        if (state == TrackingState.Amp) {
            return ArmConstants.armAmp;
        }
        if (state == TrackingState.Speaker) {
            System.out.println("[ERROR] Speaker tracking not implemented");
        }
        return 0.0;
    }

    public double getRobotRotationSpeed() {
        if (state == TrackingState.Amp) {
            return -Limelight.getTagFieldPos().yaw() / 5.0;
        }
        if (state == TrackingState.Speaker) {
            System.out.println("[ERROR] Speaker tracking not implemented");
        }
        return 0.0;
    }

    public static enum TrackingState {
        None,
        Amp,
        Speaker,
    }
}
