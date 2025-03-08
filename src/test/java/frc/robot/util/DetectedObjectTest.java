package frc.robot.util;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.Random;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.util.Vision.DetectedObject;
import frc.robot.util.Vision.DetectedObject.ObjectType;

/**
 * Tests DetectedObject
 */
public class DetectedObjectTest {

  @BeforeEach
  public void prepare() {
    DetectedObject.setDrive(null);
  }

  @AfterEach
  public void cleanup() {}

  /**
   * Tests if the objec pose is correct
   */
  @Test
  public void testObjectPose() {
    DetectedObject object = new DetectedObject(
      Units.degreesToRadians(45),
      0,
      1,
      ObjectType.NONE,
      new Transform3d(new Translation3d(0, 0, 1), new Rotation3d(0, -Math.PI/2, Math.PI/2))
    );
    Translation3d expected =  new Translation3d(Math.sqrt(2)/2, 0, Math.sqrt(2)/2+1);
    assertEquals(expected.getX(), object.pose.getX(),  0.001);
    assertEquals(expected.getY(), object.pose.getY(),  0.001);
    assertEquals(expected.getZ(), object.pose.getZ(),  0.001);
  }
  
  /**
   * Tests the position of an object when distance is not specified
   */
  @Test
  public void testObjectPoseWithoutDistance(){
    DetectedObject object = new DetectedObject(
      0,
      -Units.degreesToRadians(20),
      ObjectType.NONE,
      new Transform3d(new Translation3d(0, 0, 1), new Rotation3d(0, Units.degreesToRadians(25), 0))
    );
    Translation3d expected =  new Translation3d(1, 0, 0);
    assertEquals(expected.getX(), object.pose.getX(),  0.001);
    assertEquals(expected.getY(), object.pose.getY(),  0.001);
    assertEquals(expected.getZ(), object.pose.getZ(),  0.001);
  }

  /**
   * Tests if the object is on the ground using random offsets
   */
  @Test
  public void testObjectOnGround(){
    Random random = new Random();
    DetectedObject object = new DetectedObject(
      random.nextDouble(-Math.PI, Math.PI),
      random.nextDouble(0.001, Math.PI/4),
      ObjectType.NONE,
      new Transform3d(new Translation3d(
        random.nextDouble(0, 100),
        random.nextDouble(0, 100),
        random.nextDouble(0.1, 100)
      ), new Rotation3d(
        0, random.nextDouble(0.001, Math.PI/4), random.nextDouble(-Math.PI, Math.PI)
      ))
    );
    assertEquals(object.pose.getZ(), 0, 0.001);
  }
}
