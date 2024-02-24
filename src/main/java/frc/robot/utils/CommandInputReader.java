package frc.robot.utils;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** A class that is able to read command inputs from a POV hat */
public class CommandInputReader {
    private static CommandInputReader instance;

    private InputSequence currentSequence;
    private final int timeout;

    private int inputTimer;
    /** Create a new CommandInputReader with the given timeout.
     * @param timeout How many periods before the current input should be reset.
     */
    private CommandInputReader(int timeout) {
        currentSequence = InputSequence.None;
        this.timeout = timeout;
        inputTimer = 0;
    }

    /** Get the current instance of CommandInputReader, or create it if it does not exist.
     * @return The current instance of CommandInputReader.
     */
    public static CommandInputReader getInstance() {
        if (instance == null) {
            instance = new CommandInputReader(25);
        }
        return instance;
    }

    /** Process the given POV hat value and get the current input sequence.
     * @param pov The position POV hat value.
     * @return The current input sequence.
     */
    public InputSequence processHat(int pov) {
        if (inputTimer >= timeout) {
            currentSequence = InputSequence.None;
            inputTimer = 0;
        }

        ++inputTimer;

        SmartDashboard.putNumber("input timer", inputTimer);
        SmartDashboard.putString("input seq", currentSequence.toString());

        if (pov == Pov.HAT_NONE) {
            return currentSequence;
        }

        switch (currentSequence) {
        case None:
        {
            if (pov == Pov.HAT_DOWN) {
                currentSequence = InputSequence._2;
            }
            inputTimer = 0;
        } break;
        case _2:
        {
            if (pov == Pov.HAT_DOWN_RIGHT) {
                currentSequence = InputSequence._23;
                inputTimer = 0;
            } else if (pov == Pov.HAT_DOWN_LEFT) {
                currentSequence = InputSequence._21;
                inputTimer = 0;
            } else if (pov != Pov.HAT_DOWN) {
                currentSequence = InputSequence.None;
                inputTimer = 0;
            }
        } break;
        case _23:
        {
            if (pov == Pov.HAT_RIGHT) {
                currentSequence = InputSequence._236;
                inputTimer = 0;
            } else if (pov != Pov.HAT_DOWN_RIGHT) {
                currentSequence = InputSequence.None;
                inputTimer = 0;
            }
        } break;
        case _21:
        {
            if (pov == Pov.HAT_LEFT) {
                currentSequence = InputSequence._214;
                inputTimer = 0;
            } else if (pov != Pov.HAT_DOWN_LEFT) {
                currentSequence = InputSequence.None;
                inputTimer = 0;
            }
        } break;
        case _214:
        case _236: {} break;
        }

        return currentSequence;
    }

    /** An enum that represents command input sequences */
    public static enum InputSequence {
        None,
        _2,
        _23,
        _236,
        _21,
        _214,
    }
}
