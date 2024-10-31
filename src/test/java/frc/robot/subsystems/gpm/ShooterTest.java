package frc.robot.subsystems.gpm;

import edu.wpi.first.math.util.Units;
import lib.PolynomialRegression;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;

public class ShooterTest {
    Shooter shooter = new Shooter();
    public boolean bNoise = false;

    @BeforeEach
    public void prepare() {
    }

    @AfterEach
    public void cleanup() {
        // shut down the motors to allow more tests.
        shooter.close();
    }

    @Test
    public void motorValueTest() {
        // the free speed of a NEO Vortex is 6784.0 rpm
        assertEquals(6784.0, Units.radiansPerSecondToRotationsPerMinute(Shooter.gearbox.freeSpeedRadPerSec), 1.0);
    }

    /**
     * Check that the conversion routines are inverses of each other.
     */
    @Test
    public void conversionTest() {
        // a test rpm
        double rpmInput = 1500.0;
        // converted to a velocity
        double velocity = Shooter.shooterRPMToSpeed(rpmInput);
        
        // converting velocity back to an rpm should match the input
        assertEquals(rpmInput, Shooter.shooterSpeedToRPM(velocity), 0.0001);

        // 6000 rpm is 100 rps. circumference is pi * 4 inches. Geared up by 2.0.
        assertEquals(
            (1.0 / Shooter.gearRatio) * 100.0 * Math.PI * Units.inchesToMeters(4.0),
            Shooter.shooterRPMToSpeed(6000.0),
            0.0001);
    }

    @Test
    public void spinUpTest() {
        // shooter RPM should be at zero
//        assertEquals(0.0, shooter.getRightMotorRPM(), 0.0001);
//        assertEquals(0.0, shooter.getLeftMotorRPM(), 0.00001);
//
//        // set the target speeds to zero
//        shooter.setTargetRPM(0.0, 0.0);
//
//        // shooter PID should be satisfied
//        // satisfied is cached? Do some calculations to fill the cache.
//        shooter.leftPID.calculate(0.0);
//        shooter.rightPID.calculate(0.0);
//        assertTrue(shooter.atSetpoint());
//
//        // set the motor speed to 1000 RPM
//        shooter.setTargetRPM(1000.0, 1000.0);
//        // we should not be satisfied
//        assertFalse(shooter.atSetpoint());
//
//        // run some simulations
//        // TODO: after 3 seconds, we are still 75 RPM away!
//        // Tolerance is 80 RPM, so we are at the set point.
//        for (int i = 0; i < 150; i++) {
//            shooter.simulationPeriodic();
//            shooter.periodic();
//            // print some early values out
//            if (i < 15) {
//                System.out.println(shooter.getLeftMotorRPM());
//            }
//        }
//
//        // we should have settled down
//        assertTrue(shooter.atSetpoint());
    }

    // We have shooter data at
    // https://docs.google.com/spreadsheets/d/10JpBlUdVOniuSvXAG5M9LxhE3K0s10cQR82EBcGcMh8/edit#gid=0
    // but it needs interpretation
    // A is the measurement in Australian bananas (aka inches).
    //    so A is [inches]
    // B is the set RPM of the shooter motor
    // C = A1*(385.826772/(2*18))^(1/2)
    //     Newton tells us d = 0.5 g t^2
    //     so time to fall a distance d is
    //     sqrt(2 d / g) = t
    //       therefore 1/t = sqrt(g / (2 d))
    //     let g = 9.8 m/s/s
    //     if distance is in inches, then distance in meters is (distance * 0.0254)
    //     assume (2*18) is the (2 * d) and represents a fall of 18 inches
    //       so 1/t = sqrt(9.8 [m/s^2] * (1 [inch] / 0.0254 [m]) / (2 * 18 [inches]))
    //              = sqrt(385.82677165354330708661417322835 / (2 * 18 [s*2]))
    //     so C is exit velocity in [inches/second]
    // D = ((((C1+C2+C3)/3)/(4*PI()))*60)/2 ... is this the effective RPM?
    //     (C1+C2+C3)/3 averages Ci an gives exit velocity in [inches per second]
    //     dividing by 4 [inches] * pi gives effective shooter wheel speed in [revolutions per second]
    //     multiplying by 60 gives effective shooter wheel speed in [revolutions per minute]
    //     dividing by 2 gives effective motor RPM

    /**
     * Shooter curve fit tests.
     * <p>
     * This fit shows the coefficient of 0.64 is about correct, but it ignores the constant term.
     * <p>
     * The domain of the tests starts off at about 10 meters/second.
     * It should have started much lower to find the breakpoint.
     * We expect an exact match at low RPM with an asymptote at very high RPM.
     */
    @Test
    public void shooterCurveTest() {
        // the shooter data from the spreadsheet
        // input variable is the shooter motor RPM
        double[] rpm = {1000.0, 1000.0, 1000.0, 1500.0, 1500., 1500.0, 2000.0, 2000.0, 2000.0, 2500.0, 2500.0, 2500.0, 3000.0, 3000.0, 3000.0, 3500.0, 3500.0};
        // output variable is distance [inches] the note travelled
        double[] dist = {129.,    105.,   109.,   168.,  172.,   161.,   204.,  200.5,   204.,   255.,   270.,  278.5,   292.,   284.,   289.,   314.,   316.};
        
        // computed results
        // exit speed in inches per second
        double[] ips = new double[dist.length];
        // exit speed in meters per second
        double[] mps = new double[dist.length];
        // effective RPM
        double[] rpmE = new double[dist.length];

        // time to fall d = 18 inches
        // 18 inches in meters
        double d = Units.inchesToMeters(18.0);
        // acceleration of gravity in meters/second/second
        double g = 9.8;
        // fall time in seconds
        double t = Math.sqrt(2.0 * d / g);

        // compute the note exit velocity in inches per second from the traveled distance
        for (int i = 0; i < dist.length; i++) {
            // speed is just inches divided by time
            ips[i] = dist[i] / t;
            mps[i] = Units.inchesToMeters(ips[i]);
        }

        // compute the effective motor RPM from the exit velocity
        for (int i = 0; i < dist.length; i++) {
            // divide ips by the 4-inch wheel circumference to get effective shooter wheel rev/second,
            // multiply by 60 to get RPM, and divide by 2 get effective motor RPM
            rpmE[i] = (ips[i] / (4.0 * Math.PI)) * 60.0 / 2.0;
        }

        // calculate the regression (linear fit) for effective RPM: 0.65 rpm + 306.
        PolynomialRegression regression = new PolynomialRegression(rpm, rpmE, 1);

        if (bNoise) {
            System.out.println("Regression");
            // the constant term
            System.out.println(regression.beta(0));
            // the linear term
            System.out.println(regression.beta(1));

            // print results of the curve fit
            for (int i = 0; i < dist.length; i++) {
                System.out.printf("  %2d, %8.2f %8.2f %8.2f\n", i, rpm[i], rpmE[i], regression.predict(rpm[i]));
            }
        }

        // we can verify the fudge factor coefficient used in the Shooter class
        assertEquals(0.64, regression.beta(1), 0.02);

        if (bNoise) {
            // compare rotational energy to energy loss due to the note
            // assume no loss
            // the fits at 2000 and 3000 RPM are surprisingly good...
            //
            System.out.println("Energy storage and energy loss");
            for (int i = 0; i < dist.length; i++) {
                // rotational energy (J) at the start
                double omegaInitial = Units.rotationsPerMinuteToRadiansPerSecond(2.0 * rpm[i]);
                double energyWheelsInitial = 0.5 * omegaInitial * omegaInitial * Shooter.MOI_SHAFT;

                // rotational energy (J) at the end (assuming no slipping!)
                double omegaFinal = Units.rotationsPerMinuteToRadiansPerSecond(2.0 * rpmE[i]);
                double energyWheelsFinal = 0.5 * omegaFinal * omegaFinal * Shooter.MOI_SHAFT;

                // energy in the note
                // mass of note (game manual page 36)
                double massNote = 0.2353;
                // kinetic energy of the note
                double energyNote = 0.5 * mps[i] * mps[i] * massNote;

                System.out.printf(" %2d %8.0f %8.3f %8.3f %8.3f %8.3f\n",
                    i, rpm[i], energyWheelsInitial, energyWheelsFinal, energyNote,
                    energyWheelsInitial - energyWheelsFinal - energyNote);
            }
        }
    }
}
