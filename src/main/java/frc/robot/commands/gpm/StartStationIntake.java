package frc.robot.commands.gpm;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.IntakeConstants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.outtake.Outtake;

public class StartStationIntake extends SequentialCommandGroup {
    public StartStationIntake(Elevator elevator, Outtake outtake, Arm arm) {
		addCommands(
            new MoveElevator(elevator, ElevatorConstants.STATION_INTAKE_SETPOINT),
            new MoveArm(arm, ArmConstants.STATION_INTAKE_SETPOINT)
        );
        outtake.setMotor(0.7);
	}
}
