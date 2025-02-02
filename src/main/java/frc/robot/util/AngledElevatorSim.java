package frc.robot.util;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.NumericalIntegration;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class AngledElevatorSim extends ElevatorSim {
    private double angle;
    private boolean simulateGravity;
    private double minHeight;
    private double maxHeight;
    private double springAccel;

    /**
     * Creates a simulated angled elevator mechanism.
     *
     * @param gearbox The type of and number of motors in the elevator gearbox.
     * @param gearing The gearing of the elevator (numbers greater than 1 represent reductions).
     * @param carriageMassKg The mass of the elevator carriage.
     * @param drumRadiusMeters The radius of the drum that the elevator spool is wrapped around.
     * @param minHeightMeters The min allowable height of the elevator.
     * @param maxHeightMeters The max allowable height of the elevator.
     * @param simulateGravity Whether gravity should be simulated or not.
     * @param startingHeightMeters The starting height of the elevator.
     * @param angleRads The angle of the elevator from vertical in radians.
     * @param springForceNewtons The force of the constant force spring in Newtons. Up is positive.
     * @param measurementStdDevs The standard deviations of the measurements. Can be omitted if no
     *     noise is desired. If present must have 1 element for position.
     */
    public AngledElevatorSim(
      DCMotor gearbox,
      double gearing,
      double carriageMassKg,
      double drumRadiusMeters,
      double minHeightMeters,
      double maxHeightMeters,
      boolean simulateGravity,
      double startingHeightMeters,
      double angleRads,
      double springForceNewtons,
      double...   measurementStdDevs) {
        super(gearbox, gearing, carriageMassKg, drumRadiusMeters, minHeightMeters, maxHeightMeters, simulateGravity, startingHeightMeters, measurementStdDevs);
        angle = angleRads;
        this.simulateGravity = simulateGravity;
        minHeight = minHeightMeters;
        maxHeight = maxHeightMeters;
        springAccel = springForceNewtons/carriageMassKg;
    }

    // Copied from ElevatorSim with one difference
    /**
     * Creates a simulated elevator mechanism.
     *
     * @param gearbox The type of and number of motors in the elevator gearbox.
     * @param gearing The gearing of the elevator (numbers greater than 1 represent reductions).
     * @param carriageMassKg The mass of the elevator carriage.
     * @param drumRadiusMeters The radius of the drum that the elevator spool is wrapped around.
     * @param minHeightMeters The min allowable height of the elevator.
     * @param maxHeightMeters The max allowable height of the elevator.
     * @param simulateGravity Whether gravity should be simulated or not.
     * @param startingHeightMeters The starting height of the elevator.
     * @param measurementStdDevs The standard deviations of the measurements. Can be omitted if no
     *     noise is desired. If present must have 1 element for position.
     */
    @Override
    protected Matrix<N2, N1> updateX(Matrix<N2, N1> currentXhat, Matrix<N1, N1> u, double dtSeconds) {
        // Calculate updated x-hat from Runge-Kutta.
        var updatedXhat =
            NumericalIntegration.rkdp(
                (x, _u) -> {
                Matrix<N2, N1> xdot = m_plant.getA().times(x).plus(m_plant.getB().times(_u));
                if (simulateGravity) {
                    // This is the only line that is different
                    xdot = xdot.plus(VecBuilder.fill(0, springAccel-9.8*Math.cos(angle)));
                }
                return xdot;
                },
                currentXhat,
                u,
                dtSeconds);

        // We check for collisions after updating x-hat.
        if (wouldHitLowerLimit(updatedXhat.get(0, 0))) {
        return VecBuilder.fill(minHeight, 0);
        }
        if (wouldHitUpperLimit(updatedXhat.get(0, 0))) {
        return VecBuilder.fill(maxHeight, 0);
        }
        return updatedXhat;
    }
}
