package frc.robot.commands.gpm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.elevator.Elevator;

public class NetSetpoint extends SequentialCommandGroup {
    public NetSetpoint(Elevator elevator, Arm arm, Drivetrain drive){
        this(true, elevator, arm, drive);
    }
    public NetSetpoint(boolean chooseClosestSide, Elevator elevator, Arm arm, Drivetrain drive){
            //TODO will this spinning while elevator is going up break the robot?
        addCommands(
            new InstantCommand(()->{
                if(chooseClosestSide){
                    drive.setAlignAngle(
                        useSetpoint1(drive, chooseClosestSide) == (drive.getPose().getX() < FieldConstants.FIELD_LENGTH/2)
                        ? Math.PI / 2 : -Math.PI / 2
                    );
                    drive.setIsAlign(true);
                }
            }),
            new MoveElevator(elevator, ElevatorConstants.NET_SETPOINT),
            new ConditionalCommand(
                new MoveArm(arm, ArmConstants.ALGAE_NET_SETPOINT_1),
                new MoveArm(arm, ArmConstants.ALGAE_NET_SETPOINT_2),
                ()->useSetpoint1(drive, chooseClosestSide))
        );
    }
    private static boolean useSetpoint1(Drivetrain drive, boolean chooseClosestSide){
        boolean positive = MathUtil.angleModulus(drive.getYaw().getRadians()) > 0;
        return !chooseClosestSide || positive == (drive.getPose().getX() < FieldConstants.FIELD_LENGTH/2);
    }
}
