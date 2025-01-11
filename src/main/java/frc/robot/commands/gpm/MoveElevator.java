package frc.robot.commands.gpm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.swerve.DriveConstants;
import frc.robot.subsystems.gpm.Elevator;

public class MoveElevator extends Command {
    private Elevator elevator;
    private double setpoint;
    double accel = DriveConstants.MAX_LINEAR_ACCEL;
    public MoveElevator(Elevator elevator, double setpoint){
        this.elevator = elevator;
        this.setpoint = setpoint;
        addRequirements(elevator);
    }
    @Override
    public void initialize(){
        
        DriveConstants.MAX_LINEAR_ACCEL = 1.68213715; 
        elevator.setSetpoint(setpoint);
    }
    @Override
    public boolean isFinished(){
        return Math.abs(elevator.getPosition()-setpoint) < 0.025;  
    }
}
