package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Robot;
import frc.robot.commands.Climb.Chain;
import frc.robot.constants.miscConstants.VisionConstants.CHAIN_POSES;
import frc.robot.subsystems.Drivetrain;

public class ChainAlign extends GoToPose {
    public ChainAlign(Chain chain, Drivetrain drive){
        super(()->getChainPose(chain), drive);
    }
    private static Pose2d getChainPose(Chain chain){
        switch(chain){
            case LEFT:
                return Robot.getAlliance()==Alliance.Red?CHAIN_POSES.RED_LEFT.pose2:CHAIN_POSES.BLUE_LEFT.pose2;
            case CENTER:
                return Robot.getAlliance()==Alliance.Red?CHAIN_POSES.RED_CENTER.pose2:CHAIN_POSES.BLUE_CENTER.pose2;
            case RIGHT:
                return Robot.getAlliance()==Alliance.Red?CHAIN_POSES.RED_RIGHT.pose2:CHAIN_POSES.BLUE_RIGHT.pose2;
            default:
                return null;
        }
    }
}
