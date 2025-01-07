package frc.robot.commands.gpm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.gpm.Elevator;

public class MoveElevator extends Command {
    private Elevator elevator;
    private double setpoint;
    public MoveElevator(Elevator elevator, double setpoint){
        this.elevator = elevator;
        this.setpoint = setpoint;
        addRequirements(elevator);
    }
    @Override
    public void initialize(){
        elevator.setSetpoint(setpoint);
    }
    @Override
    public boolean isFinished(){
        return Math.abs(elevator.getPosition()-setpoint) < 0.025;
    }
}
