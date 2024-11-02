package frc.robot.commands.gpm;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.gpm.Turret;

public class TrackTarget extends Command {
    private Translation2d target;
    private Supplier<Pose2d> poseSupplier;
    private Turret turret;

    public TrackTarget(Pose2d target, Supplier<Pose2d> poseSupplier, Turret turret){
        this.target = target.getTranslation();
        this.poseSupplier = poseSupplier;
        this.turret = turret;
        addRequirements(turret);
    }

    @Override
    public void initialize(){}

    @Override
    public void execute(){
        Pose2d pose = poseSupplier.get();
        if(pose == null || target == null){
            return;
        }
        Translation2d difference = pose.getTranslation().minus(target);
        double angle = difference.getAngle().getRadians();
        double driveAngle = pose.getRotation().getRadians();
        double turretAngle = MathUtil.angleModulus(angle-driveAngle);
        turret.setAngle(Units.radiansToDegrees(turretAngle));
    }
}
