package frc.robot.subsystems.sim;

import frc.robot.subsystems.Intake;

public class SimIntake extends Intake {
    @Override
    public boolean isSimulation(){
        return true;
    }

    // Runs the Intake periodic, and then calls the simulation periodic for real robots
    @Override
    public void periodic(){
        // super.periodic();
        // if(!RobotBase.isSimulation()){
        //     simulationPeriodic();
        // }
    }
}
