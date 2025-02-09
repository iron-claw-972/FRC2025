package frc.robot.subsystems.sim;

import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.subsystems.Climb;

public class SimClimb extends Climb {
    @Override
    public boolean isSimulation(){
        return true;
    }

    // Runs the Climb periodic, and then calls the simulation periodic for real robots
    @Override
    public void periodic(){
        super.periodic();
        if(!RobotBase.isSimulation()){
            simulationPeriodic();
        }
    }
}
