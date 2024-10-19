package frc.robot.commands.gpm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ElevatorConstants;
import frc.robot.subsystems.gpm.Elevator;

public class CalibrateElevator extends Command {
    private Elevator elevator;

    public CalibrateElevator(Elevator elevator){
        this.elevator = elevator;
        addRequirements(elevator);
    }

    @Override
    public void initialize(){
        elevator.calibrate();;
    }

    @Override
    public void end(boolean interrupted){
        elevator.setSetpoint(ElevatorConstants.START_HEIGHT);
    }

    @Override
    public boolean isFinished(){
        return elevator.isCalibrated();
    }
}
