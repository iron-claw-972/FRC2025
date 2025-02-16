package frc.robot.subsystems.sim;

import frc.robot.subsystems.Indexer;

public class SimIndexer extends Indexer {
    @Override
    public boolean isSimulation(){
        return true;
    }

    // Runs the Indexer periodic, and then calls the simulation periodic for real robots
    @Override
    public void periodic(){
        // super.periodic();
        // if(!RobotBase.isSimulation()){
        //     simulationPeriodic();
        // }
    }
}
