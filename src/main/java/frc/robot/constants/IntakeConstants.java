package frc.robot.constants;

import edu.wpi.first.math.util.Units;

/**
 * Storage class for intake constants
 */
public class IntakeConstants {
    public static final double PIVOT_GEAR_RATIO = 45;
    public static final double MOMENT_OFiNERTIA = 0.600984136;
    public static final double CENTER_OF_MASS_DIST = 0.265;
    public static final double MASS = 6.45688739;
    public static final double ARM_LENGTH = CENTER_OF_MASS_DIST*2;
    public static final double DETECT_CORAL_DIST = Units.inchesToMeters(20);
}
