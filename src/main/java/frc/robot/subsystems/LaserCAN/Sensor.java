package frc.robot.subsystems.LaserCAN;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.Measurement;
import au.grapplerobotics.interfaces.LaserCanInterface.RangingMode;
import au.grapplerobotics.interfaces.LaserCanInterface.RegionOfInterest;
import au.grapplerobotics.interfaces.LaserCanInterface.TimingBudget;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IdConstants;

public class Sensor extends SubsystemBase {

    private LaserCan sensor;

    public Sensor() {
        try {
            sensor = new LaserCan(IdConstants.LASERCAN_ID);
            sensor.setRangingMode(RangingMode.SHORT);
            sensor.setTimingBudget(TimingBudget.TIMING_BUDGET_20MS);
            sensor.setRegionOfInterest(new RegionOfInterest(-4, -4, 8, 8));
            System.out.println("LaserCan initialized successfully");
        } catch (ConfigurationFailedException e) {
            System.out.println("LaserCan configuration failed: " + e.getMessage());
        } catch (Exception e) {
            System.out.println("LaserCan initialization error: " + e.getMessage());
        }
    }

    public double getDistance() {
        Measurement m = sensor.getMeasurement();

        if (m == null) {
            return Double.NaN;
        }

        double d = m.distance_mm;
        if (Double.isNaN(d) || d <= 0) {
            return Double.NaN;
        }

        return d;
    }

    public boolean detected() {
        Measurement m = sensor.getMeasurement();
    
        if (m == null) {
            SmartDashboard.putString("LaserCan", "No Measurement");
            return false;
        }
    
        double distance = m.distance_mm;

        if (Double.isNaN(distance) || distance <= 0) {
            SmartDashboard.putString("LaserCan", "No Valid Target");
            return false;
        }
    
        SmartDashboard.putNumber("LaserCan Distance (mm)", distance);
    
        // Valid measurement
        return distance <= 100;
    }
    

    @Override
    public void periodic() {
        double d = getDistance();

        if (Double.isNaN(d)) {
            SmartDashboard.putString("LaserCan Distance", "No data");
        } else {
            SmartDashboard.putNumber("LaserCan Distance (mm)", d);
        }
    }
}
