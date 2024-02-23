package frc.robot.utils;

public class CommandInputReader {
    private static CommandInputReader instance;

    private InputSequence currentSequence;
    private final int timeout;

    private int inputTimer;
    private CommandInputReader(int timeout) {
        currentSequence = InputSequence.None;
        this.timeout = timeout;
        inputTimer = 0;
    }

    public static CommandInputReader getInstance() {
        if (instance == null) {
            instance = new CommandInputReader(25);
        }
        return instance;
    }

    public InputSequence processHat(int pov) {
        if (inputTimer >= timeout) {
            currentSequence = InputSequence.None;
            inputTimer = 0;
        }

        ++inputTimer;

        if (pov == Pov.HAT_NONE) {
            return currentSequence;
        }

        switch (currentSequence) {
        case None:
        {
            if (pov == Pov.HAT_DOWN) {
                currentSequence = InputSequence._2;
                inputTimer = 0;
            }
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

    public static enum InputSequence {
        None,
        _2,
        _23,
        _236,
        _21,
        _214,
    }
}
