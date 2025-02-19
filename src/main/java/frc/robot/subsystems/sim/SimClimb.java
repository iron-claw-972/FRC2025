package frc.robot.subsystems.sim;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.subsystems.Climb;

public class SimClimb extends Climb {
    // Delete the motor to avoid CAN errors
    public SimClimb(){
        deleteMotor();
    }

    // Override some of the methods in Climb to not use the motor
    @Override
    protected double getMotorPosition(){
        return Units.radiansToRotations(climbSim.getAngleRads()) * totalGearRatio;
    }
    @Override
    protected void setMotor(double power) {}

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
