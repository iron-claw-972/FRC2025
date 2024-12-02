package frc.robot.util;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.EnumSource;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.commands.Shoot;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.constants.miscConstants.VisionConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.gpm.Arm;
import frc.robot.subsystems.PowerPanel;
import frc.robot.subsystems.gpm.Shooter;
import frc.robot.subsystems.gpm.StorageIndex;

/**
 * Shoot-While-Moving tests.
 * <p>
 * Currently disabled because the tests are creating NaNs.
 * <p>
 * The math tests should also not need to build the subsystems.
 * The math in the Shoot command is opaque.
 */
@Disabled
public class SWMTest {
    // TODO: the math tests should not require subsystems....
    // the subsystems
    //   if these are static, then the tests run faster (but the .close() calls should be removed)
    private PowerPanel powerPanel = new PowerPanel();
    private Shooter shooter = new Shooter();
    private Arm arm = new Arm(/*powerPanel*/);
    private Drivetrain drive = new Drivetrain(null);
    private StorageIndex indexer = new StorageIndex();

    /** The Shoot command */
    private Shoot sh = new Shoot(shooter, arm, drive, indexer);

    /*
     * make drivetrain in the test
     * method to set pose, reset pose
     * set chassis speed or drive() to make x,y velocities.
     */
    /**
     * Test cases for shooting-on-the-move.
     * <p>
     * These test cases have the input conditions, but they did not have the expected answers.
     * The results of running the tests have been used as the expected answers,
     * but some of those results are suspect. For example, we do not expect the
     * exit velocity to be negative.
     * <p>
     * TODO: Negative x values do not make sense on the real field
     */
    private enum TestCase {
        A(12.922, 5.537, 0, 0, 0, 0.423905, 3.140748, 15.000000),
        B(2, 1.5, 1.619, 0, 0, 0.417965, 2.017422, 15.000000),
        C(1.5, 2, 1.619, 0, 0, 0.452867, 1.956002, 15.000000) /*,
        D(1, 3, 1.619, 0, 0, 0.556990, 1.923729, 15.000000),
        
        E(2, 1.5, 1.619, 3, 0, 0.379936, 2.195832, 16.417273),
        F(-2, 1.5, 1.619, 3, 0, 0.446654, 1.312740, 14.022654),
        G(2, -1.5, 1.619, 3, 0, 0.342092, 2.032205, 16.008936),
        H(-2, -1.5, 1.619, 3, 0, 0.379173, 1.501577, 14.505448),
        I(1.5, 2, 1.619, 3, 0, 0.415474, 2.143980, 16.260643),
        J(-1.5, 2, 1.619, 3, 0, 0.477584, 1.375839, 14.189854),
        K(1.5, -2, 1.619, 3, 0, 0.346327, 1.958698, 15.803013),

        // TODO: negative vertical angle and negative exit velocity
        L(-1.5, -2, 1.7, 3, 0, -0.372748, -1.558975, -14.730002),
        M(2, 1.5, 1.7, 0, 3, 0.502500, 2.134647, 12.641822),
        N(-2, 1.5, 1.7, 0, 3, 0.498278, 0.977160, 12.674014),
        O(2, -1.5, 1.7, 0, 3, 0.449152, 1.910680, 12.368301),
        P(-2, -1.5, 1.7, 0, 3, 0.448474, 1.210690, 12.383165),
        Q(1.5, 2, 1.7, 0, 3, 0.547424, 2.060889, 12.609623),
        R(-1.5, 2, 1.7, 0, 3, 0.542072, 1.044368, 12.641980),
        S(1.5, -2, 1.7, 0, 3, 0.451044, 1.809858, 12.306013),
        T(-1.5, -2, 1.7, 0, 3, 0.450640, 1.311952, 12.316444),
        U(2, 1.5, 1.7, 3, 2, 0.418886, 2.281514, 14.968989),

        V(-2, 1.5, 1.7, 3, 2, 0.514092, 1.265134, 12.317544),
        W(2, -1.5, 1.7, 3, 2, 0.383237, 2.099102, 14.362078),
        X(-2, -1.5, 1.7, 3, 2, 0.437452, 1.489528, 12.674247),
        Y(1.5, 2, 1.7, 3, 2, 0.460166, 2.225983, 14.778710),
        Z(-1.5, 2, 1.7, 3, 2, 0.549616, 1.339441, 12.485928),
        a(1.5, -2, 1.7, 3, 2, 0.390397, 2.016765, 14.095881),
        // TODO: negative vertical angle and negative exit velocity
        b(-1.5, -2, 1.7, 3, 2, -0.429288, -1.556958, -12.888089),

        c(.2, 3, 1.7, 1, 1, 0.614164, 1.715583, 14.258364) /* */
        ;

        /** Robot X coordinate [meters] */
        double x;
        /** Robot Y coordinate [meters] */
        double y;
        /** Robot velocity in x-direction [meters/second] */
        double vx;
        /** Robot velocity in y-direction [meters/second] */
        double vy;

        // the expected answers
        /** expected vertical angle */
        double va;
        /** expected horizontal angle */
        double ha;
        /** expected exit velocity */
        double ev;

        // constructor for the test cases
        TestCase(double x, double y, double z, double vx, double vy, double va, double ha, double ev) {
            this.x = x;
            this.y = y;
            // TODO: the z value for a robot pose should be 0
            // this.z = z;
            this.vx = vx;
            this.vy = vy;

            // the answers...
            this.va = va;
            this.ha = ha;
            this.ev = ev;
        }
    };

    @AfterEach
    public void cleanup() {
        // close the subsystems
        shooter.close();
        arm.close();
        indexer.close();
        drive.close();
        powerPanel.close();
    }

    /**
     * Test if the shoot-while-moving command works.
     */
    @ParameterizedTest
    @EnumSource(TestCase.class)
    public void test_SWM(TestCase testCase) {
        // TODO: should test both the Blue and Red Alliance

        /* sh.displacement = new Pose3d(
            test_cases[idx][0],
            test_cases[idx][1],
            -test_cases[idx][2],
            new Rotation3d());
        // set v_rx
        sh.v_rx = test_cases[idx][3];
        // set v_ry
        sh.v_ry = test_cases[idx][4];
        */

        // set the robot position to (x, y) (with zero heading)
        drive.resetOdometry(new Pose2d(
            testCase.x,
            testCase.y,
            new Rotation2d()));

        // that should position the robot
        assertEquals(testCase.x, drive.getPose().getX(), 0.0001);
        assertEquals(testCase.y, drive.getPose().getY(), 0.0001);

        // set the robot velocities vx, vy and the robot heading
        drive.driveHeading(testCase.vx, testCase.vy, 0, true);
        // System.err.println("vx expected " + testCase.vx);
        // System.err.println("vx actual : " + drive.getChassisSpeeds().vxMetersPerSecond);
        // make sure the speeds were set
        assertEquals(testCase.vx, drive.getChassisSpeeds().vxMetersPerSecond, 0.0001);
        assertEquals(testCase.vy, drive.getChassisSpeeds().vyMetersPerSecond, 0.0001);

        // do the Shoot command calculation

        // initialize will set the speakerPose field from our Alliance
        sh.initialize();
        // but we want to force it to be the Blue speaker for now
        sh.speakerPose = VisionConstants.BLUE_SPEAKER_POSE;
        assertTrue(VisionConstants.BLUE_SPEAKER_POSE == sh.speakerPose);

        // Execute the Command...
        sh.execute();

        // execute() should set reasonable values for horiz_angle, vert_angle, and exit_vel
        System.err.printf("execute() returns: va %f, ha %f, and exit_vel %f\n", sh.vert_angle, sh.horiz_angle, sh.exit_vel);
        // we should not see any NaNs
        assertFalse(Double.isNaN(sh.vert_angle));
        assertFalse(Double.isNaN(sh.horiz_angle));
        assertFalse(Double.isNaN(sh.exit_vel));

        // Shoot assumes v_note = 15 m/s
        // double v_note = ShooterConstants.SHOOT_SPEED_MPS;
        assertEquals(15.0, ShooterConstants.SHOOT_SPEED_MPS, 0.001);
        // check that equation was followed (we do not know phi_v)
        // assertEquals(sh.exit_vel, v_note * Math.sin(phi_v) / Math.sin(sh.vert_angle), 0.0001);

        // now check the displacement
        Pose3d speakerPose = VisionConstants.BLUE_SPEAKER_POSE;
        // shooterHeight and shooterOffset have an additional offset because the shooter is offset from the arm, right?
        Rotation2d driveYaw = drive.getYaw();
        // TODO: the robot yaw is always zero!
        assertEquals(0.0, drive.getYaw().getRadians(), 0.001);

        // TODO: these magic numbers should not be here! They should be in the Arm and Shooter subsystems
        double armAngle = ArmConstants.standbySetpoint; // arm.getAngleRad();
        double angleToShooter = armAngle + Units.degreesToRadians(28.78);
        double shooterToPivot = Units.inchesToMeters(13.651);
        double horizontalDist = ArmConstants.PIVOT_X + shooterToPivot * Math.cos(angleToShooter);

        // calculate the rotated offset.
        double offsetX = horizontalDist * driveYaw.getCos();
        double offsetY = horizontalDist * driveYaw.getSin();

        // the displacement should be calculated from the robot pose
        System.err.println("displacement");
        System.err.println(sh.displacement);
        System.err.println(drive.getPose().getX() + offsetX - speakerPose.getX());
        System.err.println(drive.getPose().getY() + offsetY - speakerPose.getY());
        assertEquals(drive.getPose().getX() + offsetX - speakerPose.getX(), sh.displacement.getX(), 0.0001);
        assertEquals(drive.getPose().getY() + offsetY - speakerPose.getY(), sh.displacement.getY(), 0.0001);

        // the bad news from above is the displacement is about 16 meters -- essentially the whole field length!
        // fixed by going to the Blue Alliance side of the field

        // get the instantaneous velocity in the Z direction
        //    I think this should be v_note
        double vz = sh.exit_vel*Math.sin(sh.vert_angle);
        // the velocity should not be a NaN
        assertFalse(Double.isNaN(vz));

        // Caution: v_note == 15 != v_shoot == exit_vel
        //   shooter elevation is sh.vert_angle
        //   shooter azimuth is sh.horz_angle
        //   v_shoot * cos(sh.vert_angle) is velocity in the X-Y plane
        //   v_shoot * cos(sh.vert_angle) * cos(sh.horiz_angle) is velocity in the X direction
        //      but also need to add sh.v_rx to account for robot speed in X direction
        //   v_shoot * cos(sh.vert_angle) * sin(sh.horiz_angle) is velocity in the Y direction
        //      but also need to add sh.v_ry to account for robot speed in Y direction
        // time to travel the X distance = [X] / velocity-in-X-direction
        double tx= Math.abs(sh.displacement.getX()/(sh.exit_vel*Math.cos(sh.vert_angle)*Math.cos(sh.horiz_angle)+sh.v_rx));
        // time to travel the Y distance = [Y] / velocity-in-Y-direction
        double ty = Math.abs(sh.displacement.getY()/(sh.exit_vel*Math.cos(sh.vert_angle)*Math.sin(sh.horiz_angle)+sh.v_ry));
        // time to reach the Z displacement D = .getZ() ...
        //    Newton gives the height z as
        //    z = D =      v_z t - 0.5 g t^2
        //        0 = -D + v_z t - 0.5 g t^2
        //        0 =  D - v_z t + 0.5 g t^2
        //    Quadratic formula gives
        //        t = [ v_z +- sqrt(v_z^2 - 2 g D) ] / g
        //    the solution differs because D is negative (flips minus sign)
        //    the + solution should be infeasible because it will hit the hood in most cases...
        double            tz = (vz-Math.sqrt(vz*vz+19.6*sh.displacement.getZ()))/9.8;
        // empirical hack for positive solution....
        if (tx-tz > .001) tz = (vz+Math.sqrt(vz*vz+19.6*sh.displacement.getZ()))/9.8;

        // none of the times should be NaNs
        assertFalse(Double.isNaN(tx));
        assertFalse(Double.isNaN(ty));
        assertFalse(Double.isNaN(tz));
        // System.err.println("ts: " + tx + ", " + ty + ", " + tz + ", vz " + vz);

        // test the times are close
        assertEquals(tx, ty, 0.00001);
        assertEquals(tx, tz, 0.00001);

        // Compare the expected answers
        assertEquals(testCase.va, sh.vert_angle, 0.0001);
        assertEquals(testCase.ha, sh.horiz_angle, 0.0001);
        assertEquals(testCase.ev, sh.exit_vel, 0.0001);
    }

    @Test
    public void testo() {
        // 0,0 is the bottom right corner of the blue wall (m)
        // make a displacement
        Pose3d disp = new Pose3d(1, 4, .4,new Rotation3d()).relativeTo(new Pose3d(0, 0, 2.055, new Rotation3d()));

        // System.out.println(disp.getX()+","+disp.getY()+","+disp.getZ()+" done");
        // System.out.println(disp);

        // make sure the displacement is as expected
        assertEquals(1.000 - 0.000, disp.getX(), 0.00001);
        assertEquals(4.000 - 0.000, disp.getY(), 0.00001);
        assertEquals(0.400 - 2.055, disp.getZ(), 0.00001);
    }
}

