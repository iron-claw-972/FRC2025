package frc.robot.util;

import java.time.Duration;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.constants.Constants;
import frc.robot.constants.miscConstants.FieldConstants;
import frc.robot.constants.miscConstants.VisionConstants;
import frc.robot.constants.swerve.DriveConstants;

// Vision and it's commands are adapted from Iron Claw's FRC2023
public class Vision {
  // private NetworkTable m_objectDetectionTable;

  // private NetworkTableEntry m_xOffset;
  // private NetworkTableEntry m_yOffset;
  // private NetworkTableEntry m_objectDistance;
  // private NetworkTableEntry m_objectClass;
  // private NetworkTableEntry m_cameraIndex;
  
  // The field layout. Instance variable
  private AprilTagFieldLayout m_aprilTagFieldLayout;
  // A list of the cameras on the robot. TODO: Why is this nested? 
  private ArrayList<VisionCamera> m_cameras = new ArrayList<>();

  private VisionSystemSim visionSim;

  private boolean sawTag = false;

  /**
   * Creates a new instance of Vision and sets up the cameras and field layout
   */
  public Vision(ArrayList<Pair<String, Transform3d>> camList) {
    // // // Initialize object_detection NetworkTable
    // m_objectDetectionTable = NetworkTableInstance.getDefault().getTable("object_detection");

    // // From the object_detection NetworkTable, get the entries
    // m_objectDistance = m_objectDetectionTable.getEntry("distance");
    // m_xOffset = m_objectDetectionTable.getEntry("x_offset");
    // m_yOffset = m_objectDetectionTable.getEntry("y_offset");
    // m_objectClass = m_objectDetectionTable.getEntry("class");
    // m_cameraIndex = m_objectDetectionTable.getEntry("index");

    // Start NetworkTables server
    // NetworkTableInstance.getDefault().startServer();

    // Load field layout
    m_aprilTagFieldLayout = new AprilTagFieldLayout(FieldConstants.APRIL_TAGS, FieldConstants.kFieldLength, FieldConstants.kFieldWidth);

    // Sets the origin to the right side of the blue alliance wall
    m_aprilTagFieldLayout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);

    // Puts the cameras in an array list
    for (int i = 0; i < camList.size(); i++) {
      m_cameras.add(new VisionCamera(camList.get(i).getFirst(), camList.get(i).getSecond()));
    }

    if(RobotBase.isSimulation()){
      visionSim = new VisionSystemSim("Vision");
      visionSim.addAprilTags(m_aprilTagFieldLayout);
      for(VisionCamera c : m_cameras){
        PhotonCameraSim cameraSim = new PhotonCameraSim(c.camera);
        cameraSim.enableDrawWireframe(true);
        cameraSim.prop.setAvgLatencyMs(30);
        cameraSim.prop.setCalibration(720, 1280, Rotation2d.fromDegrees(78));
        visionSim.addCamera(cameraSim, c.photonPoseEstimator.getRobotToCameraTransform());
      }
    }
  }


  // /**
  //  * Get the horizontal offsets from the crosshair to the targets
  //  * @return An array of offsets in degrees
  //  */
  // public double[] getHorizontalOffset(){
  //   if(!VisionConstants.OBJECT_DETECTION_ENABLED){
  //     return new double[0];
  //   }
  //   return m_xOffset.getDoubleArray(new double[0]);
  // }

  // /**
  //  * Get the vertical offsets from the crosshair to the targets
  //  * @return An array of offsets in degrees
  //  */
  // public double[] getVerticalOffset(){
  //   if(!VisionConstants.OBJECT_DETECTION_ENABLED){
  //     return new double[0];
  //   }
  //   return m_yOffset.getDoubleArray(new double[0]);
  // }

  // /**
  //  * Get the target distances
  //  * @return Distance in meters
  //  */
  // public double[] getDistance(){
  //   if(!VisionConstants.OBJECT_DETECTION_ENABLED){
  //     return new double[0];
  //   }
  //   return m_objectDistance.getDoubleArray(new double[0]);
  // }

  // /**
  //  * Returns whether or not a valid object is detected
  //  * @return true or false
  //  */
  // public boolean validObjectDetected(){
  //   return getHorizontalOffset().length > 0;
  // }

  // /**
  //  * Returns what types of object are detected
  //  * @return The object types as a String array
  //  */
  // public long[] getDetectedObjectClass(){
  //   if(!VisionConstants.OBJECT_DETECTION_ENABLED){
  //     return new long[0];
  //   }
  //   return m_objectClass.getIntegerArray(new long[0]);
  // }

  /**
   * Gets the camera indices (which camera sees the object)
   * @return The indices as a long array (method returns long array instead of int array)
   */
  // public long[] getCameraIndex(){
  //   if(!VisionConstants.OBJECT_DETECTION_ENABLED){
  //     return new long[0];
  //   }
  //   return m_cameraIndex.getIntegerArray(new long[0]);
  // }

  /**
   * Stores all of the detected objects in an array
   * @return The array of DetectedObjects
   */
  // public DetectedObject[] getDetectedObjects(){
  //   if(!VisionConstants.OBJECT_DETECTION_ENABLED){
  //     return new DetectedObject[0];
  //   }
  //   double[] xOffset = getHorizontalOffset();
  //   double[] yOffset = getVerticalOffset();
  //   // double[] distance = getDistance();
  //   long[] objectClass = getDetectedObjectClass();
  //   // long[] cameraIndex = getCameraIndex();
  //   DetectedObject[] objects = new DetectedObject[xOffset.length];
  //   for(int i = 0; i < objects.length; i++){
  //     objects[i] = new DetectedObject(
  //       Units.degreesToRadians(xOffset[i]),
  //       Units.degreesToRadians(yOffset[i]),
  //       // distance[i],
  //       objectClass[i],
  //       // VisionConstants.OBJECT_DETECTION_CAMERAS.get((int)cameraIndex[i]).getSecond()
  //       VisionConstants.OBJECT_DETECTION_CAMERAS.get(0)
  //     );
  //   }
  //   return objects;
  // }

  /**
   * Returns the closest game piece in front of the robot
   * @param maxAngle The maximum angle from the front of the robot to use
   * @return The best DetectedObject
   */
  // public DetectedObject getBestGamePiece(double maxAngle){
  //   DetectedObject[] objects = getDetectedObjects();
  //   DetectedObject best = null;
  //   double closest = Double.POSITIVE_INFINITY;
  //   for(DetectedObject object : objects){
  //     double dist = object.getDistance();
  //     if(object.isGamePiece() && Math.abs(object.getRelativeAngle()) < maxAngle && dist < closest){
  //       closest = dist;
  //       best = object;
  //     }
  //   }
  //   return best;
  // }

  /**
   * Gets the pose as a Pose2d using PhotonVision
   * @param referencePoses The reference poses in order of preference, null poses will be skipped
   * @return The pose of the robot, or null if it can't see april tags
   */
  public Pose2d getPose2d(Pose2d... referencePoses){
    Pose2d referencePose = new Pose2d();
    for (Pose2d checkReferencePose:referencePoses){
      if (checkReferencePose != null) {
        referencePose = checkReferencePose;
        break;
      }
    }
    ArrayList<EstimatedRobotPose> estimatedPoses = getEstimatedPoses(referencePose);
    
    if (estimatedPoses.size() == 1) return estimatedPoses.get(0).estimatedPose.toPose2d();
    
    if (estimatedPoses.size() == 2) {
      return new Pose2d(
        estimatedPoses.get(0).estimatedPose.toPose2d().getTranslation()
          .plus(estimatedPoses.get(1).estimatedPose.toPose2d().getTranslation())
          .div(2),
        
        new Rotation2d(MathUtils.modulusMidpoint(
          estimatedPoses.get(0).estimatedPose.toPose2d().getRotation().getRadians(),
          estimatedPoses.get(1).estimatedPose.toPose2d().getRotation().getRadians(),
          -Math.PI, Math.PI)
        )
      );
    }
          
    //TODO: VERY LOW PRIORITY FOR FUTURE ROBOTS, make the rotation average work with more than 2 cameras
    // for(int i = 0; i < estimatedPoses.size(); i ++){
    //   translation=translation.plus(estimatedPoses.get(i).estimatedPose.toPose2d().getTranslation());
    // }

    // if(posesUsed>0){
    //   return new Pose2d(translation.div(estimatedPoses.size()), new Rotation2d());
    // }
    return null;
  }

  public AprilTagFieldLayout getAprilTagFieldLayout(){
    return m_aprilTagFieldLayout;
  }

  /**
   * Gets the pose of an april tag
   * @param id AprilTag id (1-8)
   * @return Pose3d of the AprilTag
   */
  public Pose3d getTagPose(int id){
    if(id < 1 || id > getAprilTagFieldLayout().getTags().size()){
      System.out.println("Tried to find the pose of april tag "+id);
      return null;
    }
    return getAprilTagFieldLayout().getTags().get(id-1).pose;
  }

  /**
   * Returns where it thinks the robot is
   * @param referencePose The pose to use as a reference, usually the previous robot pose
   * @return An array list of estimated poses, one for each camera that can see an april tag
   */
  public ArrayList<EstimatedRobotPose> getEstimatedPoses(Pose2d referencePose) {
    ArrayList<EstimatedRobotPose> estimatedPoses = new ArrayList<>();
    for (int i = 0; i < m_cameras.size(); i++) {
      if(VisionConstants.USE_MANUAL_CALCULATIONS){
        Pose2d pose = m_cameras.get(i).getEstimatedPose(referencePose.getRotation().getRadians());
        if(pose != null){
          try{
            EstimatedRobotPose estimatedPose = new EstimatedRobotPose(
              new Pose3d(pose.getX(), pose.getY(), 0, new Rotation3d(0, 0, pose.getRotation().getRadians())), 
              m_cameras.get(i).getTimeStamp(), 
              List.of(m_cameras.get(i).getBestTarget()),
              VisionConstants.POSE_STRATEGY
            );
            estimatedPoses.add(estimatedPose);
            if(Constants.DO_LOGGING){
              LogManager.add("Vision/camera " + i + "/estimated pose2d", () -> new Double[] {
                pose.getX(),
                pose.getY(),
                pose.getRotation().getRadians()
              }, Duration.ofSeconds(1));
            }
          }catch(Exception e){
            System.out.println(e.getStackTrace());
            DriverStation.reportWarning("EXCEPTION THROWN:", true);
          }
        }
      }else{
        Optional<EstimatedRobotPose> estimatedPose = m_cameras.get(i).getEstimatedPose(referencePose);
        // If the camera can see an april tag that exists, add it to the array list
        // April tags that don't exist might return a result that is present but doesn't have a pose
        if (estimatedPose.isPresent() && estimatedPose.get().estimatedPose != null) {
          estimatedPoses.add(estimatedPose.get());
          if(Constants.DO_LOGGING){
            LogManager.add("Vision/camera " + i + "/estimated pose2d", () -> new Double[] {
              estimatedPose.get().estimatedPose.getX(),
              estimatedPose.get().estimatedPose.getY(),
              estimatedPose.get().estimatedPose.getRotation().getZ()
            }, Duration.ofSeconds(1));
          }
        }
      }
    }
    //TODO: If poses are different, return nothing
    return estimatedPoses; 
  }

  /**
   * Updates the robot's odometry with vision
   * @param poseEstimator The pose estimator to update
   */
  public void updateOdometry(SwerveDrivePoseEstimator poseEstimator){
    // Simulate vision
    if(RobotBase.isSimulation() && VisionConstants.ENABLED_SIM){
      visionSim.update(poseEstimator.getEstimatedPosition());
    }

    sawTag = false;

    // An array list of poses returned by different cameras
    ArrayList<EstimatedRobotPose> estimatedPoses = getEstimatedPoses(poseEstimator.getEstimatedPosition());
    for (int i = 0; i < estimatedPoses.size(); i++) {
      EstimatedRobotPose estimatedPose = estimatedPoses.get(i);
      // Continue if this pose doesn't exist
      if(estimatedPose.timestampSeconds < 0 || !onField(estimatedPose.estimatedPose.toPose2d()) || Timer.getFPGATimestamp() < estimatedPose.timestampSeconds || Timer.getFPGATimestamp() > estimatedPose.timestampSeconds + 1){
        continue;
      }

      // System.out.println("\nVIsion x:" + estimatedPose.estimatedPose.getX() + "    "+ "Vision Y: " + estimatedPose.estimatedPose.getY());
      // for(PhotonTrackedTarget t : estimatedPose.targetsUsed){
      //   System.out.printf("Dist to tag %d: %.3fm\n", t.getFiducialId(), getTagPose(t.getFiducialId()).getTranslation().toTranslation2d().getDistance(poseEstimator.getEstimatedPosition().getTranslation()));
      // }
      poseEstimator.addVisionMeasurement(
        estimatedPose.estimatedPose.toPose2d(),
        estimatedPose.timestampSeconds
      );
      sawTag = true;
    }
  }

  /**
   * If vision saw any April tags last frame
   * @return If vision saw an April tag last frame
   */
  public boolean canSeeTag(){
    return sawTag;
  }

  /**
   * Enable or disable a single camera
   * @param index The camera index
   * @param enabled If it should be enabled or disabled
   */
  public void enableCamera(int index, boolean enabled){
    try{
      m_cameras.get(index).enable(enabled);
    }catch(IndexOutOfBoundsException e){
      DriverStation.reportWarning("Camera index "+index+" is out of bounds", false);
    }
  }
  /**
   * Sets the cameras to only use one April tag
   * @param id The id of the tag to use
   */
  public void onlyUse(int[] ids){
    for(VisionCamera c : m_cameras){
      c.setOnlyUse(ids);
    }
  }

  /**
   * Checks if a pose is on the field
   * @param pose The pose to check
   * @return If the pose is on the field
   */
  public static boolean onField(Pose2d pose){
    return pose!=null && pose.getX()>0 && pose.getX()<FieldConstants.kFieldLength && pose.getY()>0 && pose.getY()<FieldConstants.kFieldWidth;
  }
  
  private class VisionCamera {
    PhotonCamera camera;
    PhotonPoseEstimator photonPoseEstimator;
    Pose2d lastPose;
    double lastTimestamp = 0;
    boolean enabled = true;
    int[] onlyUse = new int[0];
  
    /**
     * Stores information about a camera
     * @param cameraName The name of the camera on PhotonVision
     * @param robotToCam The transformation from the robot to the camera
     */
    public VisionCamera(String cameraName, Transform3d robotToCam) {
      camera = new PhotonCamera(cameraName);
      photonPoseEstimator = new PhotonPoseEstimator(
        m_aprilTagFieldLayout, 
        VisionConstants.POSE_STRATEGY, 
        camera, 
        robotToCam
      );
      photonPoseEstimator.setMultiTagFallbackStrategy(VisionConstants.MULTITAG_FALLBACK_STRATEGY);
      photonPoseEstimator.setReferencePose(new Pose2d());
      lastPose = null;
    }
  
    /**
     * Gets the estimated pose from the camera
     * @param referencePose Pose to use for reference, usually the previous estimated robot pose
     * @return estimated robot pose
     */
    public Optional<EstimatedRobotPose> getEstimatedPose(Pose2d referencePose) {
      photonPoseEstimator.setReferencePose(referencePose);

      if(!enabled){
        return Optional.empty();
      }

      PhotonPipelineResult cameraResult = camera.getLatestResult();

      if(!cameraResult.hasTargets() || cameraResult.getTimestampSeconds()<0){
          return Optional.empty();
      }
      
      // if there is a target detected and the timestamp exists, 
      // check the ambiguity isn't too high
      List<PhotonTrackedTarget> targetsUsed = cameraResult.targets;
      for (int i = targetsUsed.size()-1; i >= 0; i--) {
        // found = only use is empty or this tag is in only use
        boolean found = onlyUse.length == 0;
        for(int id : onlyUse){
          if(targetsUsed.get(i).getFiducialId() == id){
            found = true;
            break;
          }
        }
        // Set found to false if it is in the list of tags to ignore
        for(int id : VisionConstants.TAGS_TO_IGNORE){
          if(targetsUsed.get(i).getFiducialId() == id){
            found = false;
            break;
          }
        }
        // Remove it from the list if it should not be used or if it has too high of an ambiguity
        if(!found || targetsUsed.get(i).getPoseAmbiguity() > VisionConstants.HIGHEST_AMBIGUITY){
          targetsUsed.remove(i);
        }
      }

      // If there are no targets, the timestamp doesn't exist, or there there is only 1 tag and the constant is set to only use 2 tags, return nothing
      if(targetsUsed.size() == 0 || cameraResult.getTimestampSeconds()<0 || targetsUsed.size()==1 && VisionConstants.ONLY_USE_2_TAGS){
        return Optional.empty();
      }

      // Set strategy to single tag if there is only 1 good tag and update
      photonPoseEstimator.setPrimaryStrategy(targetsUsed.size() > 1  ? VisionConstants.POSE_STRATEGY : VisionConstants.MULTITAG_FALLBACK_STRATEGY);
      Optional<EstimatedRobotPose> pose = photonPoseEstimator.update(cameraResult);
      
      if(pose.isPresent() && pose.get()!=null && onField(pose.get().estimatedPose.toPose2d())){
        double timestamp = getTimeStamp();

        // If the pose moved too much, don't use it
        if(lastPose==null || lastPose.getTranslation().getDistance(pose.get().estimatedPose.toPose2d().getTranslation()) > DriveConstants.kMaxSpeed*1.25*(timestamp-lastTimestamp) || timestamp < lastTimestamp){
          lastPose = pose.get().estimatedPose.toPose2d();
          lastTimestamp = timestamp;
          return Optional.empty();
        }

        // Otherwise, retrun the pose
        lastPose = pose.get().estimatedPose.toPose2d();
        lastTimestamp = timestamp;
        return pose;
      }

      // Return nothing if it gets tot his point and doesn't return anything
      return Optional.empty();
    }
    
    /**
     * Gets the pose using manual calculations
     * @param yaw The yaw of the robot to use in the calculation
     * @return estimated pose as a Pose2d
     */
    public Pose2d getEstimatedPose(double yaw){
      // Gets the best target to use for the calculations
      PhotonTrackedTarget target = camera.getLatestResult().getBestTarget();
      // Return null if the target doesn't exist or it should be ignored
      if(target==null){//|| onlyUse>0 && target.getFiducialId()!=onlyUse){
        return null;
      }
      // Return null if the id is too high or too low
      int id = target.getFiducialId();
      if(id <= 0 || id > FieldConstants.APRIL_TAGS.size()){
        return null;
      }
      // Stores target pose and robot to camera transformation for easy access later
      Pose3d targetPose = FieldConstants.APRIL_TAGS.get(id-1).pose;
      Transform3d robotToCamera = photonPoseEstimator.getRobotToCameraTransform();

      // Get the tag position relative to the robot, assuming the robot is on the ground
      Translation3d translation = new Translation3d(1, new Rotation3d(0, -Units.degreesToRadians(target.getPitch()), -Units.degreesToRadians(target.getYaw())));
      translation = translation.rotateBy(robotToCamera.getRotation());
      translation = translation.times((targetPose.getZ()-robotToCamera.getZ())/translation.getZ());
      translation = translation.plus(robotToCamera.getTranslation());
      translation = translation.rotateBy(new Rotation3d(0, 0, yaw));

      // Invert it to get the robot position relative to the April tag
      translation = translation.times(-1);
      // Multiply by a constant. I don't know why this works, but it was consistently 10% off in 2023 Fall Semester
      translation = translation.times(VisionConstants.DISTANCE_SCALE);
      // Get the field relative robot pose
      translation = translation.plus(targetPose.getTranslation());
      // Return as a Pose2d
      return new Pose2d(translation.toTranslation2d(), new Rotation2d(yaw));
    }

    /**
     * Gets the last timestamp in seconds
     * @return The timestamp in seconds
     */
    public double getTimeStamp(){
      return camera.getLatestResult().getTimestampSeconds();
    }
    
    /**
     * Gets the best target
     * @return A PhotonTrackedTarget
     */
    public PhotonTrackedTarget getBestTarget(){
      return camera.getLatestResult().getBestTarget();
    }

    /**
     * Enables or disables this camera
     * @param enable If it should be enabled or disabled
     */
    public void enable(boolean enable){
      enabled = enable;
    }
    /**
     * Sets the camera to only use 1 April tag
     * @param id The id of the tag to use, or 0 to use all
     */
    public void setOnlyUse(int[] ids){
      onlyUse = ids;
    }
  }
}
