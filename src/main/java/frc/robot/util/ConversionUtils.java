package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.constants.FieldConstants;

public class ConversionUtils {

    /**
     * @param positionCounts CANCoder Position Counts
     * @param gearRatio      Gear Ratio between CANCoder and Mechanism
     * @return Degrees of Rotation of Mechanism
     */
    public static double CANcoderToDegrees(double positionCounts, double gearRatio) {
        return positionCounts * (360.0 / (gearRatio * 4096.0));
    }

    /**
     * @param degrees   Degrees of rotation of Mechanism
     * @param gearRatio Gear Ratio between CANCoder and Mechanism
     * @return CANCoder Position Counts
     */
    public static double degreesToCANcoder(double degrees, double gearRatio) {
        return degrees / (360.0 / (gearRatio * 4096.0));
    }

    /**
     * @param positionCounts CANCoder Position Counts
     * @param gearRatio      Gear Ratio between CANCoder and Mechanism
     * @return Radians of Rotation of Mechanism
     */
    public static double CANcoderToRadians(double positionCounts, double gearRatio) {
        return Math.toRadians(CANcoderToDegrees(positionCounts, gearRatio));
    }

    /**
     * @param radians   Radians of rotation of Mechanism
     * @param gearRatio Gear Ratio between CANCoder and Mechanism
     * @return CANCoder Position Counts
     */
    public static double radiansToCANcoder(double radians, double gearRatio) {
        return degreesToCANcoder(Math.toDegrees(radians), gearRatio);
    }

    /**
     * @param positionCounts Falcon Position Counts
     * @param gearRatio      Gear Ratio between Falcon and Mechanism
     * @return Degrees of Rotation of Mechanism
     */
    public static double falconToDegrees(double positionCounts, double gearRatio) {
        return positionCounts * (360.0 / (gearRatio * 2048.0));
    }

    /**
     * @param degrees   Degrees of rotation of Mechanism
     * @param gearRatio Gear Ratio between Falcon and Mechanism
     * @return Falcon Position Counts
     */
    public static double degreesToFalcon(double degrees, double gearRatio) {
        return degrees / (360.0 / (gearRatio * 2048.0));
    }

    /**
     * @param velocityCounts Falcon Velocity Counts
     * @param gearRatio      Gear Ratio between Falcon and Mechanism (set to 1 for Falcon RPM)
     * @return RPM of Mechanism
     */
    public static double falconToRPM(double velocityCounts, double gearRatio) {
        double motorRPM = velocityCounts * (600.0 / 2048.0);
        return motorRPM / gearRatio;
    }

    /**
     * @param RPM       RPM of mechanism
     * @param gearRatio Gear Ratio between Falcon and Mechanism (set to 1 for Falcon RPM)
     * @return RPM of Mechanism
     */
    public static double RPMToFalcon(double RPM, double gearRatio) {
        double motorRPM = RPM * gearRatio;
        return motorRPM * (2048.0 / 600.0);
    }

    /**
     * @param velocitycounts Falcon Velocity Counts
     * @param circumference  Circumference of Wheel
     * @param gearRatio      Gear Ratio between Falcon and Mechanism (set to 1 for Falcon MPS)
     * @return Falcon Velocity Counts
     */
    public static double falconToMPS(double velocitycounts, double circumference, double gearRatio) {
        double wheelRPM = falconToRPM(velocitycounts, gearRatio);
        return (wheelRPM * circumference) / 60;
    }

    /**
     * @param velocity      Velocity MPS
     * @param circumference Circumference of Wheel
     * @param gearRatio     Gear Ratio between Falcon and Mechanism (set to 1 for Falcon MPS)
     * @return Falcon Velocity Counts
     */
    public static double MPSToFalcon(double velocity, double circumference, double gearRatio) {
        double wheelRPM = ((velocity * 60) / circumference);
        return RPMToFalcon(wheelRPM, gearRatio);
    }

    /**
     * @param positionCounts Falcon Position Counts
     * @param circumference  Circumference of Wheel
     * @param gearRatio      Gear Ratio between Falcon and Wheel
     * @return Meters
     */
    public static double falconToMeters(double positionCounts, double circumference, double gearRatio) {
        return positionCounts * (circumference / (gearRatio * 2048.0));
    }

    /**
     * @param meters        Meters
     * @param circumference Circumference of Wheel
     * @param gearRatio     Gear Ratio between Falcon and Wheel
     * @return Falcon Position Counts
     */
    public static double MetersToFalcon(double meters, double circumference, double gearRatio) {
        return meters / (circumference / (gearRatio * 2048.0));
    }


    /**
     * Converts between an absolute coordinate system and the pathplanner coordinate system.
     * <p>
     * Absolute coordinate system always has the origin right of the blue driver station from blue driver perspective,
     * bottom left if looking down at the field. Positive X goes toward red alliance (forward from blue driver perspective)
     * and positive Y toward red loading zone (left from blue driver perspective). The Pathplanner coordinate system has the coordinate
     * system rotated such that the origin starts right of the current driver station.
     * <p> The transformation is self-inverse, so there is no second function to convert back.
     *
     * @param pose     pose to convert
     * @param alliance alliance PathPlanner is using for their origin
     * @return converted pose
     */
    public static Pose2d absolutePoseToPathPlannerPose(Pose2d pose, Alliance alliance) {
        if (alliance == Alliance.Red) {
            return pose.relativeTo(new Pose2d(FieldConstants.FIELD_LENGTH, FieldConstants.FIELD_WIDTH, new Rotation2d(Math.PI)));
        }
        return new Pose2d(pose.getX(), pose.getY(), pose.getRotation());
    }
}
