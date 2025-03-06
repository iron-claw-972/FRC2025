package frc.robot.commands.vision;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class GoToPose2 extends Command {
    private static final double MIN_ACCEL = 2;
    private final Supplier<Pose2d> poseSupplier;
    private final Drivetrain drive;
    private Pose2d pose;
    private double vx;
    private double vy;
    private Pose2d error;

    public GoToPose2(Supplier<Pose2d> poseSupplier, Drivetrain drive){
        this.poseSupplier = poseSupplier;
        this.drive = drive;
        addRequirements(drive);
    }

    @Override
    public void initialize(){
        pose = poseSupplier.get();
        ChassisSpeeds v = drive.getChassisSpeeds();
        vx = v.vxMetersPerSecond;
        vy = v.vyMetersPerSecond;
        error = null;
    }

    @Override
    public void execute(){
        if(pose == null){
            return;
        }
        Pose2d drivePose = drive.getPose();
        error = drivePose.relativeTo(pose);
        double ax = calcAccel(vx, error.getX());
        double ay = calcAccel(vy, error.getY());
        if(Math.abs(ax) < MIN_ACCEL && Math.abs(error.getX()) > 0.01){
            ax = -Math.signum(error.getX())*MIN_ACCEL;
        }
        if(Math.abs(ay) < MIN_ACCEL && Math.abs(error.getY()) > 0.01){
            ay = -Math.signum(error.getY())*MIN_ACCEL;
        }
        vx += ax*Constants.LOOP_TIME;
        vy += ay*Constants.LOOP_TIME;
        Translation2d v = new Translation2d(vx, vy).rotateBy(pose.getRotation());
        drive.driveHeading(v.getX(), v.getY(), pose.getRotation().getRadians(), true);
    }

    @Override
    public void end(boolean interrupted){
        drive.stop();
    }

    @Override
    public boolean isFinished(){
        return pose == null || error != null && error.getTranslation().getNorm() < 0.01;
    }

    private double calcAccel(double v, double x){
        if(Math.abs(x) < 0.001 || Math.abs(Math.signum(v) - Math.signum(x)) < 0.5){
            return 0;
        }
        double a = v*v/2/x;
        double a2 = -v/Constants.LOOP_TIME;
        if(Math.abs(a2) < Math.abs(a)){
            return a2;
        }
        return a;
    }
}
