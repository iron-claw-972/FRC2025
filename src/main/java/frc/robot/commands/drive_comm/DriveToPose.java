// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands.drive_comm;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.constants.VisionConstants;
import frc.robot.constants.swerve.DriveConstants;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.util.GeomUtil;

public class DriveToPose extends Command {
  protected static boolean updateTarget = false;
  private static final double drivekP = 5.0;
  private static final double drivekD = 0.0;
  private static final double thetakP = 7.0;
  private static final double thetakD = 0.0;
  private static final double driveMaxVelocity = DriveConstants.MAX_SPEED;
  private static final double driveMaxAcceleration = 2.6;
  private static final double thetaMaxVelocity = 5.0;
  private static final double thetaMaxAcceleration = 5.0;
  private static final double driveTolerance = 0.015;
  private static final double thetaTolerance = Units.degreesToRadians(1.0);
  private static final double ffMinRadius = 0.05;
  private static final double ffMaxRadius = 0.1;

  private final Drivetrain drive;
  private final Supplier<Pose2d> target;
  private Pose2d targetPose;

  private final ProfiledPIDController driveController =
      new ProfiledPIDController(
          drivekP, 0.0, drivekD, new TrapezoidProfile.Constraints(driveMaxVelocity, driveMaxAcceleration), Constants.LOOP_TIME);
  private final ProfiledPIDController thetaController =
      new ProfiledPIDController(
          thetakP, 0.0, thetakD, new TrapezoidProfile.Constraints(thetaMaxVelocity, thetaMaxAcceleration), Constants.LOOP_TIME);

  private Translation2d lastSetpointTranslation = new Translation2d();
  private double driveErrorAbs = 0.0;
  private double thetaErrorAbs = 0.0;
  private boolean running = false;
  private Supplier<Pose2d> robot;

  private Supplier<Translation2d> linearFF = () -> Translation2d.kZero;
  private DoubleSupplier omegaFF = () -> 0.0;

  private Debouncer debouncer = new Debouncer(0.2);

  public DriveToPose(Drivetrain drive, Supplier<Pose2d> target) {
    this.drive = drive;
    this.target = target;
    robot = drive::getPose;

    // Set tolerance
    driveController.setTolerance(driveTolerance);
    thetaController.setTolerance(thetaTolerance);

    // Enable continuous input for theta controller
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(drive);
  }

  public DriveToPose(
      Drivetrain drive,
      Supplier<Pose2d> target,
      Supplier<Translation2d> linearFF,
      DoubleSupplier omegaFF) {
    this(drive, target);
    this.linearFF = linearFF;
    this.omegaFF = omegaFF;
  }

  @Override
  public void initialize() {
    drive.setVisionEnabled(VisionConstants.ENABLED_GO_TO_POSE);

    targetPose = target.get();
    Pose2d currentPose = robot.get();
    ChassisSpeeds fieldVelocity = ChassisSpeeds.fromRobotRelativeSpeeds(drive.getChassisSpeeds(), currentPose.getRotation());
    Translation2d linearFieldVelocity =
        new Translation2d(fieldVelocity.vxMetersPerSecond, fieldVelocity.vyMetersPerSecond);
    
    thetaController.reset(
        currentPose.getRotation().getRadians(), fieldVelocity.omegaRadiansPerSecond);
        lastSetpointTranslation = currentPose.getTranslation();
    
    if(targetPose != null){
        driveController.reset(
            currentPose.getTranslation().getDistance(target.get().getTranslation()),
            -linearFieldVelocity
                .rotateBy(
                    targetPose
                        .getTranslation()
                        .minus(currentPose.getTranslation())
                        .getAngle()
                        .unaryMinus())
                .getX());
        }
  }

  @Override
  public void execute() {
    running = true;

    // Get current pose and target pose
    Pose2d currentPose = robot.get();
    if(updateTarget){
        targetPose = target.get();
    }
    if(targetPose == null){
        return;
    }

    // Calculate drive speed
    double currentDistance = currentPose.getTranslation().getDistance(targetPose.getTranslation());
    double ffScaler =
        MathUtil.clamp(
            (currentDistance - ffMinRadius) / (ffMaxRadius - ffMinRadius),
            0.0,
            1.0);
    driveErrorAbs = currentDistance;
    driveController.reset(
        lastSetpointTranslation.getDistance(targetPose.getTranslation()),
        driveController.getSetpoint().velocity);
    double driveVelocityScalar =
        driveController.getSetpoint().velocity * ffScaler
            + driveController.calculate(driveErrorAbs, 0.0);
    if (currentDistance < driveController.getPositionTolerance()) driveVelocityScalar = 0.0;
    lastSetpointTranslation =
        new Pose2d(
                targetPose.getTranslation(),
                currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle())
            .transformBy(GeomUtil.toTransform2d(driveController.getSetpoint().position, 0.0))
            .getTranslation();

    // Calculate theta speed
    double thetaVelocity =
        thetaController.getSetpoint().velocity * ffScaler
            + thetaController.calculate(
                currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());
    thetaErrorAbs =
        Math.abs(currentPose.getRotation().minus(targetPose.getRotation()).getRadians());
    if (thetaErrorAbs < thetaController.getPositionTolerance()) thetaVelocity = 0.0;

    Translation2d driveVelocity =
        new Pose2d(
                new Translation2d(),
                currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle())
            .transformBy(GeomUtil.toTransform2d(driveVelocityScalar, 0.0))
            .getTranslation();

    // Scale feedback velocities by input ff
    final double linearS = linearFF.get().getNorm() * 3.0;
    final double thetaS = Math.abs(omegaFF.getAsDouble()) * 3.0;
    driveVelocity =
        driveVelocity.interpolate(linearFF.get().times(DriveConstants.MAX_SPEED), linearS);
    thetaVelocity =
        MathUtil.interpolate(
            thetaVelocity, omegaFF.getAsDouble() * DriveConstants.MAX_ANGULAR_SPEED, thetaS);

    // Command speeds
    drive.drive(driveVelocity.getX(), driveVelocity.getY(), thetaVelocity, true, false);
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
    drive.setVisionEnabled(true);
    running = false;
  }

  /** Checks if the robot is stopped at the final pose. */
  public boolean atGoal() {
    return running && (driveController.atGoal() && thetaController.atGoal() || targetPose == null);
  }

  /** Checks if the robot pose is within the allowed drive and theta tolerances. */
  public boolean withinTolerance(double driveTolerance, Rotation2d thetaTolerance) {
    return running
        && (Math.abs(driveErrorAbs) < driveTolerance
        && Math.abs(thetaErrorAbs) < thetaTolerance.getRadians()
        || targetPose == null);
  }

  @Override
  public boolean isFinished(){
    return debouncer.calculate(withinTolerance(driveTolerance, new Rotation2d(thetaTolerance)));
  }
}
