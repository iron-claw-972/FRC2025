package frc.robot.subsystems.sim;

import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.constants.ElevatorConstants;
import frc.robot.subsystems.Elevator;

public class SimElevator extends Elevator {
    // Delete motors to prvent CAN errors
    public SimElevator(){
        deleteMotors();
    }

    // Override Elevator methods to not use motors
    @Override
    protected void set(double volts){
        voltage = volts;
    }
    @Override
    protected double getRightEncoder(){
        return sim.getPositionMeters() / (2 * Math.PI * ElevatorConstants.DRUM_RADIUS)  * ElevatorConstants.GEARING;
    }
    @Override
    protected void setSimMotorPosition(double rotations) {}
    @Override
    public double getVelocity(){
        return sim.getVelocityMetersPerSecond();
    }

    @Override
    public boolean isSimulation(){
        return true;
    }

    // Runs the Elevator periodic, and then calls the simulation periodic for real robots
    @Override
    public void periodic(){
        super.periodic();
        if(!RobotBase.isSimulation()){
            simulationPeriodic();
        }
    }
}
