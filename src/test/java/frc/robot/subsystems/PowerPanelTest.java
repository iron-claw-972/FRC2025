package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.PDPSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;

/**
 * WPILib examples of battery voltage simulation are too simplified.
 * Sort out some issues with simulation and unit tests.
 * <p>
 * The BatterySim class works.
 * <p>
 * The PowerDistribution class work for both the PDP and the PDH.
 * <p>
 * The RoboRioSim class fails completely.
 * After working on some other issues, some tests started working!
 * I can set the simulated voltage.
 * I do not know why.
 * <p>
 * The RobotController class fails but some methods work.
 * I can set a voltage with RoboRioSim and then read a changed simulated voltage.
 * <p>
 * Looks like some code may work with Simulation but not Unit Tests.
 */
public class PowerPanelTest {

    /**
     * The BatterySim class just computes Ohm's law via static methods.
     * The 20 milohm value should be modified to include wire and breaker resistances leading to the PowerPanel.
     * For example, 4 feet of 6 AWG wire (0.6 milohms/foot) has a resistance of 2.4 milohms.
     * The 120 A breaker has a resistance of 0.6 milohms.
     * The Anderson SB50 connector is not clear. Maybe 0.2 milohms each for 0.4 milohms.
     */
    @Test
    public void batterySimTest() {
        // BatterySim just does some Ohm's law calculations
        // BatterySim is a final class with static methods
        // BatterySim.calculateLoadedBatteryVoltage(V, R, I)

        // default battery resistance is 20 milohms
        double R = 0.020;
        // default battery voltage is 12 volts
        double V = 12.0;
        // set the current draw
        double I = 100.0;

        // both methods should give the same answer
        assertEquals(
            BatterySim.calculateDefaultBatteryLoadedVoltage(I),
            BatterySim.calculateLoadedBatteryVoltage(V, R, I),
            0.001);

        // the voltage answer should follow Ohm's law
        assertEquals(V - I * R, BatterySim.calculateLoadedBatteryVoltage(V, R, I), 0.001);
    }

    /**
     * Test the PDP simulator object.
     * Values set by the simulator immediately change the underlying PowerDistributionn object.
     */
    @Test
    public void pdpSimTest() {
        // make a CTRE Power Distribution Panel
        //    could do new PowerDistribution(0, ModuleType.kCTRE);
        PowerDistribution pdp = new PowerDistribution();

        // make its simulator
        PDPSim pdpSim = new PDPSim(pdp);

        // check the type
        assertEquals(ModuleType.kCTRE, pdp.getType());

        // at the start, the voltage should be 12 volts
        assertEquals(12.0, pdp.getVoltage(), 0.001);

        // set a different voltage
        pdpSim.setVoltage(11.0);

        // check the new voltage
        assertEquals(11.0, pdp.getVoltage(), 0.001);

        // set some channel currents
        pdpSim.setCurrent(1, 10.0);
        pdpSim.setCurrent(2, 15.0);

        // check the individual currents
        assertEquals(10.0, pdp.getCurrent(1), 0.001);
        assertEquals(15.0, pdp.getCurrent(2), 0.001);

        // check the total current
        assertEquals(25.0, pdp.getTotalCurrent(), 0.001);

        // the switchable channel should always be false on the PDP
        assertEquals(false, pdp.getSwitchableChannel());

        // close the PDP
        pdp.close();
    }

    @Test
    public void pdhSimTest() {
        // make a REV Power Distribution Hub
        PowerDistribution pdh = new PowerDistribution(1, ModuleType.kRev);

        // make its simulator
        PDPSim pdhSim = new PDPSim(pdh);

        // check the type
        // TODO: PDH getType() FAILS!
        // assertEquals(ModuleType.kRev, pdh.getType());

        // at the start, the voltage should be 12 volts
        assertEquals(12.0, pdh.getVoltage(), 0.001);

        // set a different voltage
        pdhSim.setVoltage(11.0);

        // check the new voltage
        assertEquals(11.0, pdh.getVoltage(), 0.001);

        // set some channel currents
        pdhSim.setCurrent(1, 10.0);
        pdhSim.setCurrent(2, 15.0);

        // check the individual currents
        assertEquals(10.0, pdh.getCurrent(1), 0.001);
        assertEquals(15.0, pdh.getCurrent(2), 0.001);

        // check the total current
        assertEquals(25.0, pdh.getTotalCurrent(), 0.001);

        // one channel (channel 23) is switchwable
        // assume it is enabled.
        // TODO: FAILS! Perhaps the switchable channel is not simulated....
        // assertEquals(true, pdh.getSwitchableChannel());
        // disable the switchable channel
        pdh.setSwitchableChannel(false);
        assertEquals(false, pdh.getSwitchableChannel());
        // enable the switchable channel
        // pdh.setSwitchableChannel(true);
        // TODO: FAILS!
        // assertEquals(true, pdh.getSwitchableChannel());

        // close the PDP
        pdh.close();
    }

    // RoboRioSim.getVInVoltage() was failing!
    // OMG. It started working now! (After PDH tests?)
    @Test
    public void roboRioSimTest() {
        // set a voltage different than 12.0...
        double V = 10.8;

        // RoboRioSim is a final class with static methods
        // https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/wpilibj/simulation/RoboRioSim.html

        // trying to set the voltage first does not help...
        // EXCEPTION_ACCESS_VIOLATION (0xc0000005) at pc=0x00007ffab7bc4198, pid=27672, tid=13480
        // Unexpected exception thrown.
        // org.gradle.internal.remote.internal.MessageIOException: Could not write '/127.0.0.1:53566'.
        // Possible explanation: the method wants the Driver Station to display the simulated voltage.
        RoboRioSim.setVInVoltage(V);

        // another place to get the battery voltage
        // Unexpected exception thrown
        assertEquals(V, RoboRioSim.getVInVoltage(), 0.01);

        // Now the RobotController should report that voltage
        assertEquals(V, RobotController.getBatteryVoltage(), 0.01);
    }

    @Test
    @Disabled
    public void roboRioSimMethodsTest() {
        // These results suggest the GUI simulator must be running

        // There is a RoboRioSim.setSerialNumber()
        // fails: access violation
        RoboRioSim.setSerialNumber("12345");
        // fails: could not write /127.0.0.1:55494.
        assertEquals("12345", RoboRioSim.getSerialNumber());

        // fails: could not write /127.0.0.1:55609.
        // RoboRioSim.getTeamNumber();
    }
 
    // RobotController.getBatteryVoltage() was failing!
    // Works now! Do not know why it failed earlier.
    @Test
    public void robotControllerTest() {
        // RobotController is final class with static methods.
        // https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/wpilibj/RobotController.html

        // one place is to get the battery voltage from RobotController
        // Produces EXCEPTION_ACCESS_VIOLATION
        // Perhaps this asks the RoboRIO to report its voltage, so it should only be called in real mode?
        // Works now!
        // 
        // Simulator advice was suggesting to do RoboRioSim.setVInVoltage()
        // and then using RobotController.getBatteryVoltage().
        // .see https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/wpilibj/simulation/BatterySim.html#calculateLoadedBatteryVoltage(double,double,double...)
        // 
        assertEquals(12.0, RobotController.getBatteryVoltage(), 0.001);
    }

    // See if some other part of RobotController is functional in simulation
    @Test
    public void robotControllerMethodsTest() {
        // getting FPGA time does not fail call does not throw an exception.
        RobotController.getFPGATime();

        // Can ask for the version. 2018
        // does return HALUtil.getFPGAVersion();
        // System.out.println(RobotController.getFPGAVersion());
        assertEquals(2018, RobotController.getFPGAVersion());

        // Can ask for the Revision. 0
        // Number should be 12 bits major, 8 bits minor, 12 bits build
        // does return HALUtil.getFPGARevision();
        // System.out.println(RobotController.getFPGARevision());
        assertEquals(0, RobotController.getFPGARevision());
    }

    @Test
    public void robotControllerSerialNumberTest() {
        // make a fake serial number
        String str = "123ABC";

        // Try setting the serial number
        RoboRioSim.setSerialNumber(str);

        // May not ask for the Serial Number.
        // Crashes in native code. Probably trying to connect to the RoboRIO: HALUtil.getSerialNumber();
        // works now!
        // System.out.println(RobotController.getSerialNumber());
        assertEquals(str, RobotController.getSerialNumber());
    }

    @Test
    public void robotControllerTeamTest() {
        // Set the team number
        RoboRioSim.setTeamNumber(972);

        // Get the team number
        // Access violation
        // Now works!
        // System.out.println(RobotController.getTeamNumber());
        assertEquals(972, RobotController.getTeamNumber());
    }
   
}
