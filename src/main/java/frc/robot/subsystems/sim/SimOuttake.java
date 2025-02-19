package frc.robot.subsystems.sim;

import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.subsystems.OuttakeComp;

public class SimOuttake extends OuttakeComp {
    // Delete the motor so it doesn't cause CAN errors
    public SimOuttake(){
        deleteMotor();
    }

    // Setting the motor power should do nothing
    @Override
    protected void setMotorPrivate(double power) {}

    // Override sensor methods to not try to use the real sensor
    @Override
    public boolean coralLoaded(){
        return !dioInputLoaded.getValue();
    }
    @Override
    public boolean coralEjecting(){
        return !dioInputEjecting.getValue();
    }

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
