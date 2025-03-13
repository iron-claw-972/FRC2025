package frc.robot.commands.gpm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.ElevatorConstants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.elevator.Elevator;

public class NetSetpoint extends SequentialCommandGroup {
    public NetSetpoint(Elevator elevator, Arm arm, Drivetrain drive){
        addCommands(
            new InstantCommand(()->{
                drive.setAlignAngle(
                    useSetpoint1(drive) == (Robot.getAlliance() == Alliance.Blue)
                    ? Math.PI / 2 : -Math.PI / 2
                );
                drive.setIsAlign(true);
            }),
            new MoveElevator(elevator, ElevatorConstants.NET_SETPOINT),
            new ConditionalCommand(
                new MoveArm(arm, ArmConstants.ALGAE_NET_SETPOINT_1),
                new MoveArm(arm, ArmConstants.ALGAE_NET_SETPOINT_2),
                ()->useSetpoint1(drive))
        );
    }
    private static boolean useSetpoint1(Drivetrain drive){
        boolean positive = MathUtil.angleModulus(drive.getYaw().getRadians()) > 0;
        return positive == (Robot.getAlliance() == Alliance.Blue);
    }
}
