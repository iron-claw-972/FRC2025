package frc.robot.subsystems.sim;

import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.subsystems.Outtake;

public class SimOuttake extends Outtake {
    @Override
    public boolean isSimulation(){
        return true;
    }

    // Runs the Outtake periodic, and then calls the simulation periodic for real robots
    @Override
    public void periodic(){
        super.periodic();
        if(!RobotBase.isSimulation()){
            simulationPeriodic();
        }
    }
}
