package frc.robot.commands.gpm;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.ElevatorConstants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.outtake.Outtake;

public class StationIntake extends Command {
    private Outtake outtake;
    private Arm arm;
    private Elevator elevator;

    public StationIntake(Outtake outtake, Arm arm, Elevator elevator) {
        this.arm = arm;
        this.outtake = outtake;
        this.elevator = elevator;
        addRequirements(outtake, elevator, arm);
    }

    @Override
    public void initialize() {
        new SequentialCommandGroup(
            new MoveElevator(elevator, ElevatorConstants.STATION_INTAKE_SETPOINT),
            new MoveArm(arm, ArmConstants.STATION_INTAKE_SETPOINT)
        );
        outtake.setMotor(0.7);
    }

    @Override
    public boolean isFinished() {
        return outtake.coralLoaded();
    }

    @Override
    public void end(boolean interrupted) {
        outtake.stop();
    }


}
