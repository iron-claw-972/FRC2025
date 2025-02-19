package frc.robot.subsystems.sim;

import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.subsystems.Indexer;

public class SimIndexer extends Indexer {
    private double motorPower;

    // Delete the motor to prevent CAN errors
    public SimIndexer(){
        deleteMotor();
    }

    // This needs to set a variable instead of the actual motor power
    @Override
    public void setMotor(double power){
        motorPower = power;
    }
    // And this method needs to return that variable
    @Override
    protected double getMotorSetpoint(){
        return motorPower;
    }

    @Override
    public boolean isSimulation(){
        return true;
    }

    // Runs the Indexer periodic, and then calls the simulation periodic for real robots
    @Override
    public void periodic(){
        super.periodic();
        if(!RobotBase.isSimulation()){
            simulationPeriodic();
        }
    }
}
