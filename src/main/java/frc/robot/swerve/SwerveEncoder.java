package frc.robot.swerve;

import com.ctre.phoenix6.hardware.CANcoder;

public class SwerveEncoder {
    private CANcoder encoder;
    private double encoderOffset;
    private String position;

    public SwerveEncoder(int port, String modulePosition) {
        encoder = new CANcoder(port);
        position = modulePosition;
    }
}
