package frc.robot.swerve;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.UIConstants;
import frc.robot.utils.ParameterStore;
import frc.robot.utils.Position;
import frc.robot.utils.SimpleButton;

public class SwerveEncoder {
    private CANcoder encoder;
    private double encoderOffset;
    private String positionName;

    private int port;

    private StatusSignal<Double> rotationSignal;

    private double currentAngleRaw;
    private double currentAngleRadians;

    public SwerveEncoder(int port, String modulePosition) {
        this.port = port;
        encoder = new CANcoder(port);

        rotationSignal = encoder.getAbsolutePosition();

        positionName = modulePosition;
        encoderOffset = ParameterStore.get(positionName + "-offset", 0.0);
    }

    public void updateAngle() {
        rotationSignal.refresh();

        currentAngleRaw = rotationSignal.getValueAsDouble();
        currentAngleRadians = MathUtil.angleModulus(Math.PI * (currentAngleRaw - encoderOffset));
    }

    public Rotation2d getAngle() {
        return new Rotation2d(currentAngleRadians);
    }

    public void flip() {
        updateOffset(MathUtil.inputModulus(0.5 + encoderOffset, -0.5, 0.5));
    }

    public void updateOffset(double newOffset) {
        encoderOffset = newOffset;
        ParameterStore.set(positionName + "-offset", newOffset);
    }

    public void setupUI(String name, int column) {
        UIConstants.debug
        .addDouble(
            name + " angle",
            () -> currentAngleRadians)
        .withPosition(column, 1);

        UIConstants.tuning
        .addDouble(
            port + " angle",
            () -> currentAngleRadians)
        .withPosition(column + 1, 0);
        
        SimpleButton.createButton(
            UIConstants.tuning,
            "Zero " + port,
            new Position(column, 1),
            () -> { updateOffset(-encoderOffset); });

        UIConstants.tuning
        .addDouble(
            port + " offset",
            () -> encoderOffset)
        .withPosition(column, 0);

        SimpleButton.createButton(
            UIConstants.tuning,
            "Flip " + port,
            new Position(column + 1, 1),
            () -> { flip(); });

        SimpleButton.createButton(
            UIConstants.tuning,
            "Bump " + port + " -",
            new Position(column + 1, 1),
            () -> { updateOffset(encoderOffset - 0.01); });

        SimpleButton.createButton(
            UIConstants.tuning,
            "Bump " + port + " +",
            new Position(column + 1, 2),
            () -> { updateOffset(encoderOffset + 0.01); });
    }
}
