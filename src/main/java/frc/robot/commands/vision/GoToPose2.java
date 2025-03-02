package frc.robot.commands.vision;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Drivetrain;

public class GoToPose2 extends Command {
    private static final double MIN_ACCEL = 2;
    private final Supplier<Pose2d> poseSupplier;
    private final Drivetrain drive;
    private Pose2d pose;

    public GoToPose2(Supplier<Pose2d> poseSupplier, Drivetrain drive){
        this.poseSupplier = poseSupplier;
        this.drive = drive;
        addRequirements(drive);
    }

    @Override
    public void initialize(){
        pose = poseSupplier.get();
    }

    @Override
    public void execute(){
        if(pose == null){
            return;
        }
        Pose2d drivePose = drive.getPose();
        ChassisSpeeds v = drive.getChassisSpeeds();
        Pose2d diff = pose.relativeTo(drivePose);
        double ax = calcAccel(v.vxMetersPerSecond, diff.getX());
        double ay = calcAccel(v.vyMetersPerSecond, diff.getY());
        if(Math.abs(ax) < MIN_ACCEL && Math.abs(diff.getX()) > 0.01){
            ax = -Math.signum(ax)*MIN_ACCEL;
        }
        if(Math.abs(ay) < MIN_ACCEL && Math.abs(diff.getY()) > 0.01){
            ay = -Math.signum(ay)*MIN_ACCEL;
        }
        double vx = v.vxMetersPerSecond+ax*Constants.LOOP_TIME;
        double vy = v.vyMetersPerSecond+ay*Constants.LOOP_TIME;
        drive.driveHeading(vx, vy, pose.getRotation().getRadians(), false);
    }

    @Override
    public void end(boolean interrupted){
        drive.stop();
    }

    @Override
    public boolean isFinished(){
        return pose == null;
    }

    private double calcAccel(double v, double x){
        if(Math.abs(x) < 0.001 || Math.abs(Math.signum(v) - Math.signum(x)) > 0.5){
            return 0;
        }
        // - because we need to decelerate, so if x>0, a<0
        return -v*v/2/x;
    }
}
