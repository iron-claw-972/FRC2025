package frc.robot.commands.auto_comm;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;

import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.constants.AutoConstants;
import frc.robot.constants.swerve.DriveConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.ConversionUtils;


/**TODO: MOST CONSTRUCTORS DON'T WORK YET*/
public class PathPlannerCommand extends SequentialCommandGroup {
   public PathPlannerCommand(Drivetrain drive){
  }

  
  public PathPlannerCommand(ArrayList<PathPoint> waypoints, Drivetrain drive) {
    this(waypoints, drive, true);
  }

  public PathPlannerCommand(ArrayList<PathPoint> waypoints, Drivetrain drive, boolean useAllianceColor) {
    this(new ArrayList<PathPlannerPath>(Arrays.asList(PathPlannerPath.fromPathPoints(
      List.of(
        waypoints.get(0),
        waypoints.get(1)),
      new PathConstraints(AutoConstants.MAX_AUTO_SPEED, AutoConstants.MAX_AUTO_ACCEL, DriveConstants.kMaxAngularSpeed, DriveConstants.kMaxAngularAccel),
      new GoalEndState(0, waypoints.get(1).rotationTarget.getTarget())
    ))), 0, drive, false, useAllianceColor, true);
  }

  public PathPlannerCommand(ArrayList<PathPoint> waypoints, Drivetrain drive, boolean useAllianceColor, double maxSpeed, double maxAccel) {
    this(new ArrayList<PathPlannerPath>(Arrays.asList(PathPlannerPath.fromPathPoints(
      List.of(
        waypoints.get(0),
        waypoints.get(1)),
      new PathConstraints(maxSpeed, maxAccel, DriveConstants.kMaxAngularSpeed, DriveConstants.kMaxAngularAccel),
      new GoalEndState(0, waypoints.get(1).rotationTarget.getTarget())
    ))), 0, drive, false, useAllianceColor, true);
  }
  
  // public PathPlannerCommand(String pathGroupName, int pathIndex, Drivetrain drive) {
  //   this(PathGroupLoader.getPathGroup(pathGroupName), pathIndex, drive, true, true, false); 
  // }
  
  // public PathPlannerCommand(String pathGroupName, int pathIndex, Drivetrain drive, boolean resetPose) {
  //   this(PathGroupLoader.getPathGroup(pathGroupName), pathIndex, drive, resetPose, true, false); 
  // }

  public PathPlannerCommand(List<PathPlannerPath> pathGroup, int pathIndex, Drivetrain drive, boolean resetPose){
    this(pathGroup, pathIndex, drive, resetPose, true, false);
  }

  public PathPlannerCommand(List<Pose2d> poses, PathConstraints constraints, double endSpeed, Rotation2d endRotation, Drivetrain drive, boolean resetPose, boolean useAllianceColor, boolean isPerpetual){
    this(
      List.of(new PathPlannerPath(PathPlannerPath.bezierFromPoses(poses), constraints, new GoalEndState(endSpeed, endRotation))),
      0, drive, resetPose, useAllianceColor, isPerpetual
    );
  }

  public PathPlannerCommand(List<Pose2d> poses, Drivetrain drive, Rotation2d endRotation, double maxSpeed, double maxAccel){
    this(
      poses,
      new PathConstraints(maxSpeed, maxAccel, DriveConstants.kMaxAngularSpeed, DriveConstants.kMaxAngularAccel),
      0, endRotation, drive, false, false, false
    );
  }

  public PathPlannerCommand(List<PathPlannerPath> pathGroup, int pathIndex, Drivetrain drive, boolean resetPose, boolean useAllianceColor, boolean isPerpetual) {

    addRequirements(drive);
    if (pathIndex < 0 || pathIndex > pathGroup.size() - 1) {
      throw new IndexOutOfBoundsException("Path index out of range"); 
    }
    
    addCommands(
      new InstantCommand( () -> {
        PathPlannerPath path = pathGroup.get(pathIndex);
        if(Robot.getAlliance()==Alliance.Red){
          path.flipPath();
        }
        if (resetPose) {
          drive.resetOdometry(path.getPreviewStartingHolonomicPose());
        }
        //TODO: This might be unnecessary, maybe delete it
        if(Robot.getAlliance()==Alliance.Red){
          path.flipPath();
        }
      }),
      createSwerveControllerCommand(
        pathGroup.get(pathIndex), 
        useAllianceColor ? // Pose supplier
          () -> ConversionUtils.absolutePoseToPathPlannerPose(drive.getPose(), Robot.getAlliance()) : 
          () -> drive.getPose(), 
        ()-> drive.getChassisSpeeds(),
        (chassisSpeeds) -> { drive.setChassisSpeeds(chassisSpeeds, false); }, // chassis Speeds consumer
        AutoConstants.MAX_AUTO_SPEED,
        Math.sqrt(2)*DriveConstants.kTrackWidth/2,
        useAllianceColor,  // use Alliance color
        drive, // Requires this drive subsystem
        isPerpetual
      )
    );
  }
  
  public static FollowPathHolonomic createSwerveControllerCommand(
      PathPlannerPath path, Supplier<Pose2d> poseSupplier, 
      Supplier<ChassisSpeeds> speedSupplier,
      Consumer<ChassisSpeeds> outputChassisSpeeds, 
      double maxModuleSpeed,
      double driveBaseReadius,
      boolean useAllianceColor, Drivetrain drive, boolean isPerpetual) {
    if (isPerpetual) {
      return new PPSwerveControllerCommandPerpetual(
        path,
        poseSupplier,
        speedSupplier,
        outputChassisSpeeds,
        // TODO: Decide if we should have dynamic replanning
        new HolonomicPathFollowerConfig(maxModuleSpeed, driveBaseReadius, new ReplanningConfig(true, true)),
        useAllianceColor,
        drive
      );
    }

    return new FollowPathHolonomic(
        path,
        poseSupplier,
        speedSupplier,
        outputChassisSpeeds,
        new HolonomicPathFollowerConfig(maxModuleSpeed, driveBaseReadius, new ReplanningConfig(true, true)),
        ()-> useAllianceColor,
        drive
    );
  }
}
