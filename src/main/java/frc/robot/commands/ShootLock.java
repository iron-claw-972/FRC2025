package frc.robot.commands;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

/**
 * Shoots on the move (coast for X seconds, locking controls ver.).
 */
public class ShootLock extends Command {
    
        public static final double SETUP_TIME = 1.5; // seconds
        private final Shooter shooter;
        public final Arm arm;
        public final Drivetrain drive;
        private final StorageIndex index;

        private final Timer shootTimer = new Timer();
        private final Timer setupTimer = new Timer();

        // for testing sakes
        public double horiz_angle;
        public double vert_angle;
        public double exit_vel;
        public Pose3d displacement;
        public double v_rx;
        public double v_ry;
        // TODO: put in constants for other commands to use
        private final double REST_VEL = 4; // TODO: determine the fastest idle note-exit velocity that won't kill the
                                           // battery.
        public static final double shooterHeight = ArmConstants.ARM_LENGTH*Math.sin(ArmConstants.standbySetpoint) + ArmConstants.PIVOT_HEIGHT;
        public static final double shooterOffset = ArmConstants.PIVOT_X + ArmConstants.ARM_LENGTH * Math.cos(ArmConstants.standbySetpoint);

        public ShootLock(Shooter shooter, Arm arm, Drivetrain drivetrain, StorageIndex index) {
                this.shooter = shooter;
                this.arm = arm;
                this.drive = drivetrain;
                this.index = index;
                addRequirements(shooter, arm, index, drive);
        }

        @Override
        public void initialize() {
                // Reset the timer
                shootTimer.reset();
                shootTimer.stop();
                setupTimer.reset();
                setupTimer.start();
                
                Pose3d speakerPose = DriverStation.getAlliance().isPresent() &&
                                Robot.getAlliance() == Alliance.Red ?
                                VisionConstants.RED_SPEAKER_POSE : VisionConstants.BLUE_SPEAKER_POSE;
                // shooterHeight and shooterOffset have an additional offset because the shooter is offset from the arm, right?
                Rotation2d driveYaw = drive.getYaw();
                
                // get the drivetrain velocities
                double driveSpeed = Math.hypot(drive.getChassisSpeeds().vxMetersPerSecond, drive.getChassisSpeeds().vyMetersPerSecond);
                double heading = driveYaw.getRadians() + Math.atan2(drive.getChassisSpeeds().vyMetersPerSecond, drive.getChassisSpeeds().vxMetersPerSecond);
                v_rx = driveSpeed * Math.cos(heading);
                v_ry = driveSpeed * Math.sin(heading);

                // where we will be when we take the shot.
                displacement = new Pose3d(
                        drive.getPose().getX() + shooterOffset * driveYaw.getCos()+SETUP_TIME*v_rx,
                        drive.getPose().getY() + shooterOffset * driveYaw.getSin()+SETUP_TIME*v_ry,
                        shooterHeight,
                        new Rotation3d(
                        0,
                        ShooterConstants.ANGLE_OFFSET - arm.getAngleRad(),
                        Math.PI + driveYaw.getRadians()))
                        .relativeTo(speakerPose);
                System.err.println(displacement.getX()+" " +
                                displacement.getY()+" " +
                                displacement.getZ()+" " +
                                        v_rx+" " +
                                        v_ry+" "+
                                        heading+" "+
                                        drive.getChassisSpeeds().vxMetersPerSecond+" "+
                                        drive.getChassisSpeeds().vyMetersPerSecond
                );
                // TODO: Figure out what v_note is empirically
                double v_note = 10;

                // X distance to speaker
                double x = Math.sqrt((displacement.getX() * displacement.getX())
                                // Y distance to speaker
                                + displacement.getY() * displacement.getY());
                // height (sorry that it's called y)
                double y = displacement.getZ();
                // Basic vertical angle calculation (static robot)
                double phi_v = Math.atan(Math.pow(v_note, 2) / 9.8 / x * (1 - Math.sqrt(1
                                + 19.6 / Math.pow(v_note, 2) * (y - 4.9 * x * x / Math.pow(v_note, 2)))));
                System.err.println("*pv " + phi_v);
                // Angle to goal
                double phi_h = Math.atan(displacement.getY()/ displacement.getX());
                // flip angle -- atan's range is (-pi/2, pi/2) so we flip it if we should be going left
                if (displacement.getX()>=0) phi_h += Math.PI;
                System.err.println("*ph " + phi_h);
                double theta_h = Math.atan((v_note * Math.cos(phi_v) * Math.sin(phi_h) - v_ry) / (v_note * Math.cos(phi_v) * Math.cos(phi_h) - v_rx));
                // flip angle -- atan's range is (-pi/2, pi/2) so we flip it if we should be going left
                if (displacement.getX()>=0) theta_h += Math.PI;
                // random quirk that using -v_rx, -v_ry works instead of +v_rx, +v_ry
                double theta_v = Math.atan(
                                (v_note * Math.sin(phi_v) * Math.cos(theta_h)) /
                                (v_note * Math.cos(phi_v) * Math.cos(phi_h) - v_rx));
                double v_shoot = v_note * Math.sin(phi_v) / Math.sin(theta_v);
                horiz_angle = theta_h;
                vert_angle = theta_v;
                exit_vel = v_shoot;
                System.err.println(horiz_angle);
                System.err.println(vert_angle);
                System.err.println(exit_vel);

                arm.setAngle(ShooterConstants.ANGLE_OFFSET - theta_v);
                // use driveheading with x, y speed (keep same), angle;
                drive.driveHeading(v_rx, v_ry, Math.PI+theta_h, true);
                // Set the outtake velocity
                shooter.setTargetVelocity(v_shoot);

        }
        private boolean has_elapsed=false;
        @Override
        public void execute() {
                if (setupTimer.hasElapsed(SETUP_TIME-StorageIndexConstants.ejectShootTimeout/2)){
                // if the actual shot were to happen at around halfway through the span from indexer to shooter being done.
                    if (has_elapsed){
                        index.ejectIntoShooter();
                        shootTimer.start();
                    }
                    //SmartDashboard.putBoolean("ShootLock ready?",arm.atSetpoint() && shooter.atSetpoint() && drive.atAlignAngle());
                    has_elapsed = true;
                }
        }

        @Override
        public boolean isFinished() {
                return shootTimer.hasElapsed(StorageIndexConstants.ejectShootTimeout);
        }

        @Override
        public void end(boolean interrupted) {
                shooter.setTargetVelocity(REST_VEL);
                arm.setAngle(ArmConstants.standbySetpoint);
                index.stopIndex();
        }
}
