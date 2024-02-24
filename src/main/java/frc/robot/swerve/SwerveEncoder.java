package frc.robot.swerve;

import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.UIConstants;
import frc.robot.utils.ParameterStore;
import frc.robot.utils.Position;
import frc.robot.utils.SimpleButton;

/** A class representing a swerve drive encoder */
public class SwerveEncoder {
    private CANcoder encoder;
    private double encoderOffset;
    private String positionName;

    private int port;

    private double currentAngleRaw;
    private double currentAngleRadians;

    /** Create a new SwerveEncoder on the given port with the given module name.
     * @param port The port of the encoder.
     * @param modulePosition The name of the module this encoder is attached to.
     */
    public SwerveEncoder(int port, String modulePosition) {
        this.port = port;
        encoder = new CANcoder(port);

        positionName = modulePosition;
        encoderOffset = ParameterStore.get(positionName + "-offset", 0.0);
    }

    /** Read the angle of the module from the encoder and update internal values. */
    public void updateAngle() {
        currentAngleRaw = encoder.getPosition().getValueAsDouble();
        currentAngleRadians = MathUtil.angleModulus((currentAngleRaw + encoderOffset) * SwerveConstants.turnEncoderScaleFactor);
    }

    /** Get the current rotation of the module. updateAngle() should be called before this to ensure the angle is up-to-date.
     * @return The current rotation of the module.
     */
    public Rotation2d getAngle() {
        return new Rotation2d(currentAngleRadians);
    }

    /** Flip the encoder offset by 180 degrees */
    public void flip() {
        updateOffset(MathUtil.inputModulus(0.5 + encoderOffset, -0.5, 0.5));
    }

    /** Change the encoder offset.
     * @param newOffset The new encoder offset to use.
     */
    public void updateOffset(double newOffset) {
        encoderOffset = newOffset;
        ParameterStore.set(positionName + "-offset", newOffset);
    }

    /** Initialize the encoder's UI on Shuffleboard.
     * @param name The name of the module this encoder is attached to.
     * @param column The column to place UI elements on.
     */
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
            new Position(column, 2),
            () -> { updateOffset(encoderOffset - 0.01); });

        SimpleButton.createButton(
            UIConstants.tuning,
            "Bump " + port + " +",
            new Position(column + 1, 2),
            () -> { updateOffset(encoderOffset + 0.01); });
    }
}
