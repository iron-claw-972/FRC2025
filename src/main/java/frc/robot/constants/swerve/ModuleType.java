package frc.robot.constants.swerve;

/**
 * Represents the type for a module on the robot.
 * <p/>
 * IDs:
 * 0 - FRONT_LEFT
 * 1 - FRONT_RIGHT
 * 2 - BACK_LEFT
 * 3 - BACK_RIGHT
 */
public enum ModuleType {
    FRONT_LEFT,
    FRONT_RIGHT,
    BACK_LEFT,
    BACK_RIGHT,
    NONE;

    public final byte id;

    ModuleType() {
        this.id = id();
    }

    private byte id() {
        if (this == NONE)
            return -1;
        // This is a trick that relies on the order the enums are defined.
        return (byte) this.ordinal();
    }
}