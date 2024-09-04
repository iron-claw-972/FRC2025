package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.miscConstants.VisionConstants.CHAIN_POSES;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.gpm.Arm;

public class Climb extends SequentialCommandGroup {
    public enum Chain{
        LEFT, RIGHT, CENTER;
    }

    private Pose2d pose1;
    private Pose2d pose2;
    private double tolerance1 = Units.inchesToMeters(2);
    private double tolerance2 = Units.inchesToMeters(2);

    public Climb(Chain chain, Drivetrain drive, Arm arm){
        addCommands(
            new InstantCommand(()->getPoses(chain)),
            new ParallelCommandGroup(
                arm==null?new DoNothing():new ArmToPos(arm, ArmConstants.preClimbSetpoint),
                new GoToPose(()->pose1, drive).until(
                    ()->drive.getPose().getTranslation().getDistance(pose1.getTranslation())<tolerance1)
            ),
            new GoToPose(()->pose2, drive).until(
                ()->drive.getPose().getTranslation().getDistance(pose2.getTranslation())<tolerance2),
            arm==null?new DoNothing():new ArmToPos(arm, ArmConstants.climbSetpoint)
        );
    }

    private void getPoses(Chain chain){
        boolean red = Robot.getAlliance() == Alliance.Red;
        CHAIN_POSES poses = null;
        switch(chain){
            case LEFT:
                poses = red?CHAIN_POSES.RED_LEFT:CHAIN_POSES.BLUE_LEFT;
                break;
            case CENTER:
                poses = red?CHAIN_POSES.RED_CENTER:CHAIN_POSES.BLUE_CENTER;
                break;
            case RIGHT:
                poses = red?CHAIN_POSES.RED_RIGHT:CHAIN_POSES.BLUE_RIGHT;
                break;
            default:
                System.err.println("That chain doesn't exist");
                return;
        }
        pose1 = poses.pose1;
        pose2 = poses.pose2;
    }
}
