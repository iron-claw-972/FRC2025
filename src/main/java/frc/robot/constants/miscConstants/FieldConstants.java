package frc.robot.constants.miscConstants;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;

public class FieldConstants {
  /** Width of the field [meters] */
  public static final double kFieldLength = Units.inchesToMeters(54*12 + 3.25);
  /** Height of the field [meters] */
  public static final double kFieldWidth = Units.inchesToMeters(26*12 + 11.25);

  /** 
   * ArrayList of April Tags to use when the field layout is not available.
   * <p>
   * Obtain AprilTag ID i with APRIL_TAGS.get(i-1).
   */
  public static final ArrayList<AprilTag> APRIL_TAGS = new ArrayList<AprilTag>(List.of(
    new AprilTag(1, new Pose3d(Units.inchesToMeters(593.68), Units.inchesToMeters(9.68), Units.inchesToMeters(53.38), new Rotation3d(0, 0, 2*Math.PI/3))),
    new AprilTag(2, new Pose3d(Units.inchesToMeters(637.21), Units.inchesToMeters(34.79), Units.inchesToMeters(53.38), new Rotation3d(0, 0, 2*Math.PI/3))),
    new AprilTag(3, new Pose3d(Units.inchesToMeters(652.73), Units.inchesToMeters(196.17), Units.inchesToMeters(57.13), new Rotation3d(0, 0, Math.PI))),
    new AprilTag(4, new Pose3d(Units.inchesToMeters(652.73), Units.inchesToMeters(218.42), Units.inchesToMeters(57.13), new Rotation3d(0, 0, Math.PI))),
    new AprilTag(5, new Pose3d(Units.inchesToMeters(578.77), Units.inchesToMeters(323.00), Units.inchesToMeters(53.38), new Rotation3d(0, 0, -Math.PI/2))),
    new AprilTag(6, new Pose3d(Units.inchesToMeters(72.5), Units.inchesToMeters(323.00), Units.inchesToMeters(53.38), new Rotation3d(0, 0, -Math.PI/2))),
    new AprilTag(7, new Pose3d(Units.inchesToMeters(-1.50), Units.inchesToMeters(218.42), Units.inchesToMeters(57.13), new Rotation3d(0, 0, 0))),
    new AprilTag(8, new Pose3d(Units.inchesToMeters(-1.50), Units.inchesToMeters(196.17), Units.inchesToMeters(57.13), new Rotation3d(0, 0, 0))),
    new AprilTag(9, new Pose3d(Units.inchesToMeters(14.02), Units.inchesToMeters(34.79), Units.inchesToMeters(53.38), new Rotation3d(0, 0, Math.PI/3))),
    new AprilTag(10, new Pose3d(Units.inchesToMeters(57.54), Units.inchesToMeters(9.68), Units.inchesToMeters(53.38), new Rotation3d(0, 0, Math.PI/3))),
    new AprilTag(11, new Pose3d(Units.inchesToMeters(468.69), Units.inchesToMeters(146.19), Units.inchesToMeters(52.00), new Rotation3d(0, 0, -Math.PI/3))),
    new AprilTag(12, new Pose3d(Units.inchesToMeters(468.69), Units.inchesToMeters(177.10), Units.inchesToMeters(52.00), new Rotation3d(0, 0, Math.PI/3))),
    new AprilTag(13, new Pose3d(Units.inchesToMeters(441.74), Units.inchesToMeters(161.62), Units.inchesToMeters(52.00), new Rotation3d(0, 0, Math.PI))),
    new AprilTag(14, new Pose3d(Units.inchesToMeters(209.48), Units.inchesToMeters(161.62), Units.inchesToMeters(52.00), new Rotation3d(0, 0, 0))),
    new AprilTag(15, new Pose3d(Units.inchesToMeters(182.73), Units.inchesToMeters(177.10), Units.inchesToMeters(52.00), new Rotation3d(0, 0, 2*Math.PI/3))),
    new AprilTag(16, new Pose3d(Units.inchesToMeters(182.73), Units.inchesToMeters(146.19), Units.inchesToMeters(52.00), new Rotation3d(0, 0, -2*Math.PI/3)))
  ));
}
