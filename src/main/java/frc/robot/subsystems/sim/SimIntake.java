package frc.robot.subsystems.sim;

import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.subsystems.Intake;

public class SimIntake extends Intake {
    // Delete motors to prevent CAN errors
    public SimIntake(){
        deleteMotors();
    }

    // Setting the speed should do nothing except update isMoving
    @Override
    public void setSpeed(double power) {
        isMoving = Math.abs(power) > 0.01;
    }
    @Override
    protected void setPivotMotor(double power) {}

    @Override
    public boolean isSimulation(){
        return true;
    }

    // Runs the Intake periodic, and then calls the simulation periodic for real robots
    @Override
    public void periodic(){
        super.periodic();
        if(!RobotBase.isSimulation()){
            simulationPeriodic();
        }
    }
}
