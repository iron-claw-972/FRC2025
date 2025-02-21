package frc.robot.subsystems.sim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.constants.swerve.DriveConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.Vision;

public class SimDrivetrain extends Drivetrain {
    private double yaw = DriveConstants.STARTING_HEADING.getDegrees();

    // Delete the pigeon to prevent CAN errors. Motors are deleted in ModuleSim
    public SimDrivetrain(Vision vision){
        super(vision);
        deletePigeon();
    }

    // Override methods to not use the pigeon
    @Override
    protected double getPigeonYaw(){
        return yaw;
    }
    @Override
    protected void addSimYaw(double yaw){
        this.yaw += yaw;
    }
    @Override
    public double getAngularRate(int id){
        return 0;
    }
    @Override
    public void resetOdometry(Pose2d pose){
        super.resetOdometry(pose);
        yaw = pose.getRotation().getDegrees();
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
