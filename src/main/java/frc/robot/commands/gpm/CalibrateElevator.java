package frc.robot.commands.gpm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ElevatorConstants;
import frc.robot.subsystems.gpm.Elevator;

public class CalibrateElevator extends Command {
    private Elevator elevator;
    private double start;
    private boolean movingUp;

    public CalibrateElevator(Elevator elevator){
        this.elevator = elevator;
        addRequirements(elevator);
    }

    @Override
    public void initialize(){
        movingUp = true;
        start = elevator.getPosition();
    }

    @Override
    public void execute(){
        double position = elevator.getPosition();
        if(position - start > ElevatorConstants.BOTTOM_LIMIT_SWITCH_HEIGHT - ElevatorConstants.MIN_HEIGHT){
            movingUp = false;
        }
        elevator.setSetpoint(position + (movingUp ? 0.02 : -0.02));
    }

    @Override
    public void end(boolean interrupted){
        elevator.setSetpoint(ElevatorConstants.START_HEIGHT);
    }

    @Override
    public boolean isFinished(){
        return elevator.getBottomLimitSwitch() || elevator.getTopLimitSwitch();
    }

    // This command should not be interrupted by other commands
    @Override
    public InterruptionBehavior getInterruptionBehavior(){
        return InterruptionBehavior.kCancelIncoming;
    }
}
