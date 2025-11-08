package frc.robot.subsystems.LaserCAN;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.Measurement;
import au.grapplerobotics.interfaces.LaserCanInterface.RangingMode;
import au.grapplerobotics.interfaces.LaserCanInterface.RegionOfInterest;
import au.grapplerobotics.interfaces.LaserCanInterface.TimingBudget;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.constants.IdConstants;

public class Sensor extends SubsystemBase{
    private LaserCan sensor;
        public Sensor(){
            sensor = new LaserCan(IdConstants.LASERCAN_ID);
            try{
                sensor.setRangingMode(RangingMode.SHORT);
                sensor.setTimingBudget(TimingBudget.TIMING_BUDGET_20MS);
                sensor.setRegionOfInterest(new RegionOfInterest(-4, -4, 8, 8));
            }
            catch (ConfigurationFailedException e){
                System.out.println("error");
            }
        }
    public double getDistance(){
        Measurement measurement = sensor.getMeasurement();
        //System.out.println(measurement); 
        return measurement.distance_mm;
    }
    public Measurement getMeasurement(){
        return sensor.getMeasurement();
    }
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Distance", getDistance());
        System.out.println(getDistance());
    }
    public boolean detected(){
        Measurement m = RobotContainer.sensor.getMeasurement();
        if (m == null) {
            return false;
        }
        double d = m.distance_mm;
        SmartDashboard.putString("LaserCan", m.toString());
        if (Double.isNaN(d) || d <= 0) {
            return true;
        }
        if (d <= 100) return true;
        else return false;
    }
}
