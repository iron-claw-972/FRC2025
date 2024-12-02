package frc.robot.constants;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.ArrayList;

import org.junit.jupiter.api.Test;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.constants.miscConstants.FieldConstants;
import frc.robot.util.Vision;

/**
 * Tests if all of the April tags are in the right spot
 */
public class AprilTagPoseTest {
  /**
   * Tests if all of the April tags are in the right spot
   */
  @Test
  public void testTagPoses() {
    // construct the vision instance
    //   makes the field layout
    Vision vision = new Vision(new ArrayList<Pair<String, Transform3d>>());

    // we should have 16 tags
    assertEquals(16, FieldConstants.APRIL_TAGS.size());
    assertEquals(16, vision.getAprilTagFieldLayout().getTags().size());

    // check each tag in the field layout
    for (int i = 0; i < vision.getAprilTagFieldLayout().getTags().size(); i++) {
      // the expected tagId. The ArrayList is zero-based and our tags start at 1, so the tagId is i+1.
      int tagId = i + 1;

      // get the poses from the two sources
      // from the ArrayList<AprilTag> source
      AprilTag apriltag1 = FieldConstants.APRIL_TAGS.get(i);
      Pose3d p1 = apriltag1.pose;
      // from the vision source
      Pose3d p2 = vision.getTagPose(tagId);

      // check the tag id in the ArrayList
      assertEquals(tagId, apriltag1.ID);

      // make sure the points match
      assertEquals(p1.getX(), p2.getX(), 0.0001);
      assertEquals(p1.getY(), p2.getY(), 0.0001);
      assertEquals(p1.getZ(), p2.getZ(), 0.0001);

      // make sure the rotations match
      assertEquals(p1.getRotation().getX(), p2.getRotation().getX(), 0.0001);
      assertEquals(p1.getRotation().getY(), p2.getRotation().getY(), 0.0001);
      assertEquals(p1.getRotation().getZ(), p2.getRotation().getZ(), 0.0001);
    }
  }
}
