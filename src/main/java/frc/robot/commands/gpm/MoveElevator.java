package frc.robot.commands.gpm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;

/**
 * Moves the elevator to a position
 */
public class MoveElevator extends Command {
    private Elevator elevator;
    private double setpoint;

    /**
     * Creates a command to move the elevator to a position
     * 
     * @param elevator The elevator subsystem
     * @param setpoint The setpoint to move to
     */
    public MoveElevator(Elevator elevator, double setpoint) {
        this.elevator = elevator;
        this.setpoint = setpoint;
        if(elevator != null){
            addRequirements(elevator);
        }
    }

    /**
     * Sets the elevator setpoint
     */
    @Override
    public void initialize() {
        if(elevator != null){
            elevator.setSetpoint(setpoint);
        }
    }

    /**
     * Returns whether the elevator is at the setpoint
     * 
     * @return True if the elevator is within about 1 inch of the setpoint, false
     *         otherwise
     */
    @Override
    public boolean isFinished() {
        return elevator == null || elevator.atSetpoint();
    }
}
