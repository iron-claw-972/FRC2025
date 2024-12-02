package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.commands.gpm.PrepareShooter;
import frc.robot.commands.gpm.ShootKnownPos;
import frc.robot.commands.gpm.ShootKnownPos.ShotPosition;
import frc.robot.constants.ShooterConstants;
import frc.robot.constants.miscConstants.VisionConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.gpm.Arm;
import frc.robot.subsystems.gpm.Shooter;
import frc.robot.subsystems.gpm.StorageIndex;

public class StageShoot extends SequentialCommandGroup {
    private static final double tolerance = 0.025;
    private static final Supplier<Pose2d> pose = ()->Robot.getAlliance() == Alliance.Red ? 
        VisionConstants.RED_STAGE_SHOOT_POSE :
        VisionConstants.BLUE_STAGE_SHOOT_POSE;


    public StageShoot(Drivetrain drive, Arm arm, Shooter shooter, StorageIndex index){
        addCommands(
            new PrepareShooter(shooter, Shooter.addSlip(Shooter.shooterSpeedToRPM(ShooterConstants.SHOOT_SPEED_MPS))),
            new GoToPose(pose, drive).until(()->
                pose.get().getTranslation().getDistance(drive.getPose().getTranslation()) < tolerance
            ),
            new ParallelDeadlineGroup(
                new ShootKnownPos(shooter, arm, index, ShotPosition.STAGE_ISH),
                new GoToPosePID(pose, drive)
            )
        );
    }
}
