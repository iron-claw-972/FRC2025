package frc.robot.subsystems.sim;

import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.Vision;

public class SimDrivetrain extends Drivetrain {
    public SimDrivetrain(Vision vision){
        super(vision);
    }

    @Override
    public boolean isSimulation(){
        return true;
    }

    // Runs the Drivetrain periodic, and then calls the simulation periodic for real robots
    @Override
    public void periodic(){
        super.periodic();
        if(!RobotBase.isSimulation()){
            simulationPeriodic();
        }
    }
}
