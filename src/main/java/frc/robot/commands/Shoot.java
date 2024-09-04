package frc.robot.commands;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.constants.StorageIndexConstants;
import frc.robot.constants.miscConstants.VisionConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.gpm.Arm;
import frc.robot.subsystems.gpm.Shooter;
import frc.robot.subsystems.gpm.StorageIndex;
import frc.robot.util.EqualsUtil;

//00 is the bottom right corner of blue wall in m
/**
 * Shoots on the move (instantaneous velocities and pose ver.).
 */

// TODO make the version with delay have correct math
public class Shoot extends Command {
        // the subsystems
        private final Shooter shooter;
        private final Arm arm;
        private final Drivetrain drive;
        private final StorageIndex index;
        private final Timer shootTimer = new Timer();

        // for testing sakes
        public double X,ANG,Y;

        /** Speaker pose determined when the Shoot command runs initialize() */
        public Pose3d speakerPose;

        // These are the AprilTags on the speakers. Used to restrict search.
        private static final int[] aprilTagsSpeaker = {3, 4, 7, 8};
        // The empty set to disable useOnly.
        private static final int[] aprilTagsNull = {};

        // public for testing sakes; package frc.robot.util looks at these values; those tests should be elsewhere...
        public double horiz_angle;
        public double vert_angle;
        public double exit_vel;
        public Pose3d displacement;
        public double v_rx;
        public double v_ry;
        // TODO: put in constants for other commands to use
        
        private final double REST_VEL = 0;
        // TODO: determine the fastest idle note-exit velocity that won't kill the battery.

        private boolean shooting = false;

        public Shoot(Shooter shooter, Arm arm, Drivetrain drivetrain, StorageIndex index) {
                this.shooter = shooter;
                this.arm = arm;
                this.drive = drivetrain;
                this.index = index;
                addRequirements(shooter, arm, index);
        }

        @Override
        public void initialize() {
                // Reset the timer
                shootTimer.reset();
                shootTimer.stop();

                // Enable alignment mode on the drivetrain
                drive.setIsAlign(true);

                // only use the April Tags on the speakers
                // April Tags 3, 4, 7, and 8 are the speaker tags on the Alliance Wall.
                // this is a backdoor to vision...
                drive.onlyUseTags(aprilTagsSpeaker);

                // we are not shooting yet
                shooting = false;

                // the alliance determines which speaker we target
                // the speakerPose need not be calculated every execute; do it in initialize()
                speakerPose = (Robot.getAlliance() == Alliance.Red) ?
                                VisionConstants.RED_SPEAKER_POSE : VisionConstants.BLUE_SPEAKER_POSE;
        }

        @Override
        public void execute() {
                // TODO: positive x displacement -> left of speaker only for Blue Alliance
                // Positive x displacement means we are to the left of the speaker
                // TODO: doesn't positive y displacement mean we are above the speaker?
                // Positive y displacement means we are below the speaker.
                // shooterHeight and shooterOffset have an additional offset because the shooter is offset from the arm, right?
                // get the direction the robot is facing
                Rotation2d driveYaw = drive.getYaw();

                // calculate the displacement to the speaker
                //   actually, this calculates the displacement **from** the speaker to the robot, so Z value is negative...
                // TODO: 22.7 inches - speaker.z should be negative? speakerPose.getZ() is 80.5 inches.
                // TODO: these magic numbers should not be here! They should be in the Arm and Shooter subsystems
                double angleToShooter = arm.getAngleRad()+Units.degreesToRadians(28.78);
                double shooterToPivot = Units.inchesToMeters(13.651);
                double horizontalDist = ArmConstants.PIVOT_X + shooterToPivot * Math.cos(angleToShooter);
                // Set displacement to speaker
                displacement = new Pose3d(
                        drive.getPose().getX() + horizontalDist * driveYaw.getCos()-speakerPose.getX(),
                        drive.getPose().getY() + horizontalDist * driveYaw.getSin()-speakerPose.getY(),
                        // shooterHeight-speakerPose.getZ(),
                        ArmConstants.PIVOT_HEIGHT + shooterToPivot * Math.sin(angleToShooter) - speakerPose.getZ(),
                        new Rotation3d(
                        0,
                        ShooterConstants.ANGLE_OFFSET - arm.getAngleRad(),
                        // PI + yaw because we are shooting out the stern
                        Math.PI + driveYaw.getRadians()));
                        // .relativeTo(speakerPose);
                        //.times(-1);
                
                // get the drivetrain velocities
                double driveSpeed = Math.hypot(drive.getChassisSpeeds().vxMetersPerSecond, drive.getChassisSpeeds().vyMetersPerSecond);
                double heading = driveYaw.getRadians() + Math.atan2(drive.getChassisSpeeds().vyMetersPerSecond, drive.getChassisSpeeds().vxMetersPerSecond);
                v_rx = driveSpeed * Math.cos(heading);
                v_ry = driveSpeed * Math.sin(heading);
                //System.err.println(displacement.getX()+" " +
                //                 displacement.getY()+" " +
                //                 displacement.getZ()+" " +
                //                         v_rx+" " +
                //                         v_ry+" "+
                //                         heading+" "+
                //                         drive.getChassisSpeeds().vxMetersPerSecond+" "+
                //                         drive.getChassisSpeeds().vyMetersPerSecond
                // );

                // TODO: Figure out what v_note is empirically
                double v_note = ShooterConstants.SHOOT_SPEED_MPS;

                // X distance to speaker (along the floor to center of speaker)
                double x = Math.hypot(displacement.getX(), displacement.getY());
                // height (sorry that it's called y)
                // TODO: but y is negative from above
                double y = displacement.getZ();
                Y=y;
                // Basic vertical angle calculation (static robot)
                double phi_v = Math.atan(Math.pow(v_note, 2) / 9.8 / x * (1 - Math.sqrt(1
                                + 19.6 / Math.pow(v_note, 2) * (y - 4.9 * x * x / Math.pow(v_note, 2)))));
                //System.err.println("*pv " + phi_v);

                // Angle to goal
                // TODO: isn't this calculation simplified with atan2()?
                double phi_h = Math.atan(displacement.getY()/ displacement.getX());
                // flip angle
                if (displacement.getX()>=0) phi_h += Math.PI;
                //System.err.println("*ph " + phi_h);

                // TODO: isn't this calculation simplified with atan2()?
                double theta_h = Math.atan((v_note * Math.cos(phi_v) * Math.sin(phi_h) - v_ry) / (v_note * Math.cos(phi_v) * Math.cos(phi_h) - v_rx));
                // flip angle
                if (displacement.getX()>=0) theta_h += Math.PI;
                
                // random quirk that using -v_rx, -v_ry works instead of +v_rx, +v_ry
                // theta_h conversion (i.e. pi-theta_h if necessary)
                // if the mirrored angle is the same-ish direction??? logic may break at high
                // v_rx and v_ry but don't worry about it
                /*
                if (Math.signum(Math.sin(theta_h)) != Math.signum(Math.sin(phi_h))
                                || Math.signum(Math.cos(theta_h)) != Math.signum(Math.cos(theta_h))) {
                        theta_h += Math.PI;
                }
                */
                double theta_v = Math.atan(
                                (v_note * Math.sin(phi_v) * Math.cos(theta_h)) /
                                (v_note * Math.cos(phi_v) * Math.cos(phi_h) - v_rx));
                // also here
                double v_shoot = v_note * Math.sin(phi_v) / Math.sin(theta_v);

                // save the results
                horiz_angle = theta_h;
                theta_v += Units.degreesToRadians(ArmConstants.armFudgeFactor);
                vert_angle = theta_v;
                ANG = ShooterConstants.ANGLE_OFFSET - theta_v;
                exit_vel = v_shoot;
                // System.err.println(horiz_angle);
                // System.err.println(vert_angle);
                // System.err.println(exit_vel);

                // set the shooter angle
                // TODO: Arm should have an arm.setShooterAngle() method.
                arm.setAngle(ShooterConstants.ANGLE_OFFSET - theta_v);

                // theta_h is relative to the horizontal, so
                // drive.setAlignAngle switches from theta_h to pi/2-theta_h
                // depending on if it's relative to the horizontal or the vertical.
                // Drivetrain angle is relative to positive x (toward red side)

                // Sets the angle to align to for the drivetrain, uses driveHeading in DefaultDriveCommand
                drive.setAlignAngle(Math.PI + theta_h); // would only pause rotational

                // Set the outtake velocity
                shooter.setTargetVelocity(v_shoot);

                // System.out.println("Arm Setpoint: "+arm.atSetpoint());
                // System.out.println("Shooter Setpoint: "+shooter.atSetpoint());
                // System.out.println("drive Setpoint: "+drive.atAlignAngle());
                // TODO: Make this commented out if statement work (arm and shooter weren't getting to setpoint)
                // if (arm.atSetpoint() && shooter.atSetpoint() && drive.atAlignAngle() && sawTag || shooting) {
                // report the conditions needed to start shooting
                // TODO: do not use epsilonEquals(); use arm.atSetpoint()
                // The shooter should not be worried about PID Tolerance here; that belongs in Arm.
                // SmartDashboard.putBoolean("arm setpoint", EqualsUtil.epsilonEquals(arm.getAngleRad(), ShooterConstants.ANGLE_OFFSET - theta_v, Units.degreesToRadians(1)));
                // SmartDashboard.putBoolean("shooter at setpoint", shooter.atSetpoint());
                // SmartDashboard.putBoolean("drive setpoint", drive.atAlignAngle());
                // SmartDashboard.putBoolean("saw tag", sawTag);

                if (!shooting && EqualsUtil.epsilonEquals(arm.getAngleRad(), ShooterConstants.ANGLE_OFFSET - (theta_v), Units.degreesToRadians(1 /* 4, 1 */)) && 
                 shooter.atSetpoint() && drive.atAlignAngle()) {
                        // remember we are now shooting
                        shooting = true;
                        // push the note into the shooter
                        index.ejectIntoShooter();
                        // start the shooting timer
                        shootTimer.start();
                        //System.out.println("DONE");
                }
        }

        @Override
        public boolean isFinished() {
                return shootTimer.hasElapsed(StorageIndexConstants.ejectShootTimeout);
        }

        @Override
        public void end(boolean interrupted) {
                // System.out.println("x " + X+" y "+Y+" ang "+ANG + " actual " + arm.getAngleRad());
                
                // slow down/turn off the shooter
                shooter.setTargetVelocity(REST_VEL);
                shooter.resetPID();

                // use normal driver controls
                drive.setIsAlign(false);

                // stow the arm
                arm.setAngle(ArmConstants.stowedSetpoint);

                // stop feeding the shooter
                index.stopIndex();

                // set onlyUseTags to the empty set
                drive.onlyUseTags(aprilTagsNull);
        }
}
