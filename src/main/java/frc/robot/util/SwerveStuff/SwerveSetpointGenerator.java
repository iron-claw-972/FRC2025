// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.util.SwerveStuff;

import static frc.robot.util.EqualsUtil.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import frc.robot.constants.Constants;
import frc.robot.constants.swerve.DriveConstants;
import frc.robot.util.EqualsUtil;
import frc.robot.util.GeomUtil;

/**
 * "Inspired" by FRC team 254. See the license file in the root directory of this project.
 *
 * <p>Takes a prior setpoint (ChassisSpeeds), a desired setpoint (from a driver, or from a path
 * follower), and outputs a new setpoint that respects all of the kinematic constraints on module
 * rotation speed and wheel velocity/acceleration. By generating a new setpoint every iteration, the
 * robot will converge to the desired setpoint quickly while avoiding any intermediate state that is
 * kinematically infeasible (and can result in wheel slip or robot heading drift as a result).
 */

public class SwerveSetpointGenerator {
  private final SwerveDriveKinematics kinematics= DriveConstants.KINEMATICS;
  private final Translation2d[] moduleLocations = DriveConstants.MODULE_LOCATIONS;

  /**
   * Check if it would be faster to go to the opposite of the goal heading (and reverse drive
   * direction).
   *
   * @param prevToGoal The rotation from the previous state to the goal state (i.e.
   *     prev.inverse().rotateBy(goal)).
   * @return True if the shortest path to achieve this rotation involves flipping the drive
   *     direction.
   */
  private boolean flipHeading(Rotation2d prevToGoal) {
    return Math.abs(prevToGoal.getRadians()) > Math.PI / 2.0;
  }

  private double unwrapAngle(double ref, double angle) {
    double diff = angle - ref;
    if (diff > Math.PI) {
      return angle - 2.0 * Math.PI;
    } else if (diff < -Math.PI) {
      return angle + 2.0 * Math.PI;
    } else {
      return angle;
    }
  }

  @FunctionalInterface
  private interface Function2d {
    double f(double x, double y);
  }

  /**
   * Find the root of the generic 2D parametric function 'func' using the regula falsi technique.
   * This is a pretty naive way to do root finding, but it's usually faster than simple bisection
   * while being robust in ways that e.g. the Newton-Raphson method isn't.
   *
   * @param func The Function2d to take the root of.
   * @param x_0 x value of the lower bracket.
   * @param y_0 y value of the lower bracket.
   * @param f_0 value of 'func' at x_0, y_0 (passed in by caller to save a call to 'func' during
   *     recursion)
   * @param x_1 x value of the upper bracket.
   * @param y_1 y value of the upper bracket.
   * @param f_1 value of 'func' at x_1, y_1 (passed in by caller to save a call to 'func' during
   *     recursion)
   * @param iterations_left Number of iterations of root finding left.
   * @return The parameter value 's' that interpolating between 0 and 1 that corresponds to the
   *     (approximate) root.
   */
  private double findRoot(
      Function2d func,
      double x_0,
      double y_0,
      double f_0,
      double x_1,
      double y_1,
      double f_1,
      int iterations_left) {
    if (iterations_left < 0 || epsilonEquals(f_0, f_1)) {
      return 1.0;
    }
    var s_guess = Math.max(0.0, Math.min(1.0, -f_0 / (f_1 - f_0)));
    var x_guess = (x_1 - x_0) * s_guess + x_0;
    var y_guess = (y_1 - y_0) * s_guess + y_0;
    var f_guess = func.f(x_guess, y_guess);
    if (Math.signum(f_0) == Math.signum(f_guess)) {
      // 0 and guess on same side of root, so use upper bracket.
      return s_guess
          + (1.0 - s_guess)
              * findRoot(func, x_guess, y_guess, f_guess, x_1, y_1, f_1, iterations_left - 1);
    } else {
      // Use lower bracket.
      return s_guess
          * findRoot(func, x_0, y_0, f_0, x_guess, y_guess, f_guess, iterations_left - 1);
    }
  }

  protected double findSteeringMaxS(
      double x_0,
      double y_0,
      double f_0,
      double x_1,
      double y_1,
      double f_1,
      double max_deviation,
      int max_iterations) {
    f_1 = unwrapAngle(f_0, f_1);
    double diff = f_1 - f_0;
    if (Math.abs(diff) <= max_deviation) {
      // Can go all the way to s=1.
      return 1.0;
    }
    double offset = f_0 + Math.signum(diff) * max_deviation;
    Function2d func =
        (x, y) -> {
          return unwrapAngle(f_0, Math.atan2(y, x)) - offset;
        };
    return findRoot(func, x_0, y_0, f_0 - offset, x_1, y_1, f_1 - offset, max_iterations);
  }

  protected double findDriveMaxS_254version(
      double x_0,
      double y_0,
      double f_0,
      double x_1,
      double y_1,
      double f_1,
      double max_vel_step,
      int max_iterations) {
    double diff = f_1 - f_0;
    if (Math.abs(diff) <= max_vel_step) {
      // Can go all the way to s=1.
      return 1.0;
    }
    double offset = f_0 + Math.signum(diff) * max_vel_step;
    Function2d func =
        (x, y) -> {
          return Math.hypot(x, y) - offset;
        };
    return findRoot(func, x_0, y_0, f_0 - offset, x_1, y_1, f_1 - offset, max_iterations);
  }

  /**
   * Limits the acceleration in all directions.
   * This is different from findDriveMaxS because it includes the acceleration perpendicular to the wheel as it rotates.
   * Given the same velocity step, this will return a lower S value than findDriveMaxS.
   * @param x_0 The initial x velocity
   * @param y_0 The initial y velocity
   * @param x_1 The final x velocity
   * @param y_1 The final y velocity
   * @param max_vel_step The maxiumum amount the velocity can change this frame
   * @param max_iterations The maximum number of iterations to use in findRoot
   * @return The maximum interpolation value
   */
  protected double findAccelerationMaxS(
      double x_0,
      double y_0,
      double x_1,
      double y_1,
      double max_vel_step,
      int max_iterations) {
    double dist = Math.hypot(x_1-x_0, y_1-y_0);
    if(dist <= max_vel_step){
      return 1;
    }
    return Math.max(0.0, Math.min(1.0, max_vel_step / dist));
  }

  protected double findDriveMaxS(
      double x_0, double y_0, double x_1, double y_1, double max_vel_step) {
    // Derivation:
    // Want to find point P(s) between (x_0, y_0) and (x_1, y_1) where the
    // length of P(s) is the target T. P(s) is linearly interpolated between the
    // points, so P(s) = (x_0 + (x_1 - x_0) * s, y_0 + (y_1 - y_0) * s).
    // Then,
    //     T = sqrt(P(s).x^2 + P(s).y^2)
    //   T^2 = (x_0 + (x_1 - x_0) * s)^2 + (y_0 + (y_1 - y_0) * s)^2
    //   T^2 = x_0^2 + 2x_0(x_1-x_0)s + (x_1-x_0)^2*s^2
    //       + y_0^2 + 2y_0(y_1-y_0)s + (y_1-y_0)^2*s^2
    //   T^2 = x_0^2 + 2x_0x_1s - 2x_0^2*s + x_1^2*s^2 - 2x_0x_1s^2 + x_0^2*s^2
    //       + y_0^2 + 2y_0y_1s - 2y_0^2*s + y_1^2*s^2 - 2y_0y_1s^2 + y_0^2*s^2
    //     0 = (x_0^2 + y_0^2 + x_1^2 + y_1^2 - 2x_0x_1 - 2y_0y_1)s^2
    //       + (2x_0x_1 + 2y_0y_1 - 2x_0^2 - 2y_0^2)s
    //       + (x_0^2 + y_0^2 - T^2).
    //
    // To simplify, we can factor out some common parts:
    // Let l_0 = x_0^2 + y_0^2, l_1 = x_1^2 + y_1^2, and
    // p = x_0 * x_1 + y_0 * y_1.
    // Then we have
    //   0 = (l_0 + l_1 - 2p)s^2 + 2(p - l_0)s + (l_0 - T^2),
    // with which we can solve for s using the quadratic formula.

    double l_0 = x_0 * x_0 + y_0 * y_0;
    double l_1 = x_1 * x_1 + y_1 * y_1;
    double sqrt_l_0 = Math.sqrt(l_0);
    double diff = Math.sqrt(l_1) - sqrt_l_0;
    if (Math.abs(diff) <= max_vel_step) {
      // Can go all the way to s=1.
      return 1.0;
    }

    double target = sqrt_l_0 + Math.copySign(max_vel_step, diff);
    double p = x_0 * x_1 + y_0 * y_1;

    // Quadratic of s
    double a = l_0 + l_1 - 2 * p;
    double b = 2 * (p - l_0);
    double c = l_0 - target * target;
    double root = Math.sqrt(b * b - 4 * a * c);

    // Check if either of the solutions are valid
    // Won't divide by zero because it is only possible for a to be zero if the
    // target velocity is exactly the same or the reverse of the current
    // velocity, which would be caught by the difference check.
    double s_1 = (-b + root) / (2 * a);
    if (isValidS(s_1)) {
      return s_1;
    }
    double s_2 = (-b - root) / (2 * a);
    if (isValidS(s_2)) {
      return s_2;
    }

    // Since we passed the initial max_vel_step check, a solution should exist,
    // but if no solution was found anyway, just don't limit movement
    return 1.0;
  }

  protected static boolean isValidS(double s) {
    return Double.isFinite(s) && s >= 0 && s <= 1;
  }

  /**
   * Generate a new setpoint.
   *
   * @param limits The kinematic limits to respect for this setpoint.
   * @param centerOfMassHeight The height of the robot's center of mass, in meters, off the ground.
   *     This assumes that the center of mass is in the center of the robot in the x and y directions.
   *     If tipping is not a potential problem this year, set this to 0.
   * @param prevSetpoint The previous setpoint motion. Normally, you'd pass in the previous
   *     iteration setpoint instead of the actual measured/estimated kinematic state.
   * @param desiredState The desired state of motion, such as from the driver sticks or a path
   *     following algorithm.
   * @param dt The loop time.
   * @return A Setpoint object that satisfies all of the KinematicLimits while converging to
   *     desiredState quickly.
   */
  public SwerveSetpoint generateSetpoint(
      final ModuleLimits limits,
      double centerOfMassHeight,
      final SwerveSetpoint prevSetpoint,
      ChassisSpeeds desiredState,
      double dt) {
    final Translation2d[] modules = moduleLocations;

    SwerveModuleState[] desiredModuleState = kinematics.toSwerveModuleStates(desiredState);
    // Make sure desiredState respects velocity limits.
    if (limits.maxDriveVelocity() > 0.0) {
      SwerveDriveKinematics.desaturateWheelSpeeds(desiredModuleState, limits.maxDriveVelocity());
      desiredState = kinematics.toChassisSpeeds(desiredModuleState);
    }

    // Special case: desiredState is a complete stop. In this case, module angle is arbitrary, so
    // just use the previous angle.
    boolean need_to_steer = true;
    if (EqualsUtil.GeomExtensions.epsilonEquals(GeomUtil.toTwist2d(desiredState),new Twist2d())) {
      need_to_steer = false;
      for (int i = 0; i < modules.length; ++i) {
        desiredModuleState[i].angle = prevSetpoint.moduleStates()[i].angle;
        desiredModuleState[i].speedMetersPerSecond = 0.0;
      }
    }

    // For each module, compute local Vx and Vy vectors.
    double[] prev_vx = new double[modules.length];
    double[] prev_vy = new double[modules.length];
    Rotation2d[] prev_heading = new Rotation2d[modules.length];
    double[] desired_vx = new double[modules.length];
    double[] desired_vy = new double[modules.length];
    Rotation2d[] desired_heading = new Rotation2d[modules.length];
    boolean all_modules_should_flip = true;
    for (int i = 0; i < modules.length; ++i) {
      prev_vx[i] =
          prevSetpoint.moduleStates()[i].angle.getCos()
              * prevSetpoint.moduleStates()[i].speedMetersPerSecond;
      prev_vy[i] =
          prevSetpoint.moduleStates()[i].angle.getSin()
              * prevSetpoint.moduleStates()[i].speedMetersPerSecond;
      prev_heading[i] = prevSetpoint.moduleStates()[i].angle;
      if (prevSetpoint.moduleStates()[i].speedMetersPerSecond < 0.0) {
        prev_heading[i] = prev_heading[i].rotateBy(Rotation2d.fromRadians(Math.PI));
      }
      desired_vx[i] =
          desiredModuleState[i].angle.getCos() * desiredModuleState[i].speedMetersPerSecond;
      desired_vy[i] =
          desiredModuleState[i].angle.getSin() * desiredModuleState[i].speedMetersPerSecond;
      desired_heading[i] = desiredModuleState[i].angle;
      if (desiredModuleState[i].speedMetersPerSecond < 0.0) {
        desired_heading[i] = desired_heading[i].rotateBy(Rotation2d.fromRadians(Math.PI));
      }
      if (all_modules_should_flip) {
        double required_rotation_rad =
            Math.abs(prev_heading[i].unaryMinus().rotateBy(desired_heading[i]).getRadians());
        if (required_rotation_rad < Math.PI / 2.0) {
          all_modules_should_flip = false;
        }
      }
    }
    if (all_modules_should_flip
        && !EqualsUtil.GeomExtensions.epsilonEquals(GeomUtil.toTwist2d(prevSetpoint.chassisSpeeds()),new Twist2d())
        && !EqualsUtil.GeomExtensions.epsilonEquals(GeomUtil.toTwist2d(desiredState),new Twist2d())) {
      // It will (likely) be faster to stop the robot, rotate the modules in place to the complement
      // of the desired
      // angle, and accelerate again.
      return generateSetpoint(limits, centerOfMassHeight, prevSetpoint, new ChassisSpeeds(), dt);
    }

    // Compute the deltas between start and goal. We can then interpolate from the start state to
    // the goal state; then
    // find the amount we can move from start towards goal in this cycle such that no kinematic
    // limit is exceeded.
    double dx = desiredState.vxMetersPerSecond - prevSetpoint.chassisSpeeds().vxMetersPerSecond;
    double dy = desiredState.vyMetersPerSecond - prevSetpoint.chassisSpeeds().vyMetersPerSecond;
    double dtheta =
        desiredState.omegaRadiansPerSecond - prevSetpoint.chassisSpeeds().omegaRadiansPerSecond;

    // 's' interpolates between start and goal. At 0, we are at prevState and at 1, we are at
    // desiredState.
    double min_s = 1.0;

    // In cases where an individual module is stopped, we want to remember the right steering angle
    // to command (since
    // inverse kinematics doesn't care about angle, we can be opportunistically lazy).
    List<Optional<Rotation2d>> overrideSteering = new ArrayList<>(modules.length);
    // Enforce steering velocity limits. We do this by taking the derivative of steering angle at
    // the current angle,
    // and then backing out the maximum interpolant between start and goal states. We remember the
    // minimum across all modules, since
    // that is the active constraint.
    final double max_theta_step = dt * limits.maxSteeringVelocity();
    for (int i = 0; i < modules.length; ++i) {
      if (!need_to_steer) {
        overrideSteering.add(Optional.of(prevSetpoint.moduleStates()[i].angle));
        continue;
      }
      overrideSteering.add(Optional.empty());
      if (epsilonEquals(prevSetpoint.moduleStates()[i].speedMetersPerSecond, 0.0)) {
        // If module is stopped, we know that we will need to move straight to the final steering
        // angle, so limit based
        // purely on rotation in place.
        if (epsilonEquals(desiredModuleState[i].speedMetersPerSecond, 0.0)) {
          // Goal angle doesn't matter. Just leave module at its current angle.
          overrideSteering.set(i, Optional.of(prevSetpoint.moduleStates()[i].angle));
          continue;
        }

        var necessaryRotation =
            prevSetpoint.moduleStates()[i].angle.unaryMinus().rotateBy(desiredModuleState[i].angle);
        if (flipHeading(necessaryRotation)) {
          necessaryRotation = necessaryRotation.rotateBy(Rotation2d.fromRadians(Math.PI));
        }
        // getRadians() bounds to +/- Pi.
        final double numStepsNeeded = Math.abs(necessaryRotation.getRadians()) / max_theta_step;

        if (numStepsNeeded <= 1.0) {
          // Steer directly to goal angle.
          overrideSteering.set(i, Optional.of(desiredModuleState[i].angle));
          // Don't limit the global min_s;
          continue;
        } else {
          // Adjust steering by max_theta_step.
          overrideSteering.set(
              i,
              Optional.of(
                  prevSetpoint.moduleStates()[i].angle.rotateBy(
                      Rotation2d.fromRadians(
                          Math.signum(necessaryRotation.getRadians()) * max_theta_step))));
          min_s = 0.0;
          continue;
        }
      }
      if (min_s == 0.0) {
        // s can't get any lower. Save some CPU.
        continue;
      }

      final int kMaxIterations = 8;
      double s =
          findSteeringMaxS(
              prev_vx[i],
              prev_vy[i],
              prev_heading[i].getRadians(),
              desired_vx[i],
              desired_vy[i],
              desired_heading[i].getRadians(),
              max_theta_step,
              kMaxIterations);
      min_s = Math.min(min_s, s);
    }

    // Enforce drive wheel acceleration limits.
    final double max_vel_step = dt * limits.maxDriveAcceleration();
    final double max_vel_step_2 = dt * limits.staticFriction() * Constants.GRAVITY_ACCELERATION;
    for (int i = 0; i < modules.length; ++i) {
      if (min_s == 0.0) {
        // No need to carry on.
        break;
      }
      double vx_min_s =
          min_s == 1.0 ? desired_vx[i] : (desired_vx[i] - prev_vx[i]) * min_s + prev_vx[i];
      double vy_min_s =
          min_s == 1.0 ? desired_vy[i] : (desired_vy[i] - prev_vy[i]) * min_s + prev_vy[i];
      // Find the max s for this drive wheel. Search on the interval between 0 and min_s, because we
      // already know we can't go faster
      // than that.
      final int kMaxIterations = 10;
      double s = min_s*findDriveMaxS(prev_vx[i], prev_vy[i], vx_min_s, vy_min_s, max_vel_step);
      
      // Limit the overall acceleration of this wheel
      double s2 = min_s*findAccelerationMaxS(prev_vx[i], prev_vy[i], vx_min_s, vy_min_s, max_vel_step_2, kMaxIterations);

      min_s = Math.min(Math.min(min_s, s), s2);
    }

    if(centerOfMassHeight > 0.02){
      // Limit the acceleration in the x and y directions separately based on the center of mass.
      // To make the torque on the robot 0, we can assume all of the mass is on the back wheel, where the front is the direction the robot is accelerating toward
      // Torque is equal to the force times the component of the radius perpendicular to the force
      // T = torque, m = mass, a = acceleration, g = gravity acceleration, x = distance from center to wheel
      // T = mgx - mah = 0
      // a = gx/h
      double maxAccel = Constants.GRAVITY_ACCELERATION*(DriveConstants.TRACK_WIDTH/2)/centerOfMassHeight;
      // Limit based on this calculated value
      // x and y are limited separately because, when tipping in a diagonal direction, the distance is longer
      double xAccel = Math.abs(desiredState.vxMetersPerSecond - prevSetpoint.chassisSpeeds().vxMetersPerSecond) / dt;
      double yAccel = Math.abs(desiredState.vyMetersPerSecond - prevSetpoint.chassisSpeeds().vyMetersPerSecond) / dt;
      if(!epsilonEquals(xAccel, 0)){
        double s = maxAccel / xAccel;
        min_s = Math.min(min_s, s);
      }
      if(!epsilonEquals(yAccel, 0)){
        double s = maxAccel / yAccel;
        min_s = Math.min(min_s, s);
      }
    }

    ChassisSpeeds retSpeeds =
        new ChassisSpeeds(
            prevSetpoint.chassisSpeeds().vxMetersPerSecond + min_s * dx,
            prevSetpoint.chassisSpeeds().vyMetersPerSecond + min_s * dy,
            prevSetpoint.chassisSpeeds().omegaRadiansPerSecond + min_s * dtheta);
    var retStates = kinematics.toSwerveModuleStates(retSpeeds);
    for (int i = 0; i < modules.length; ++i) {
      final var maybeOverride = overrideSteering.get(i);
      if (maybeOverride.isPresent()) {
        var override = maybeOverride.get();
        if (flipHeading(retStates[i].angle.unaryMinus().rotateBy(override))) {
          retStates[i].speedMetersPerSecond *= -1.0;
        }
        retStates[i].angle = override;
      } 
      final var deltaRotation =
          prevSetpoint.moduleStates()[i].angle.unaryMinus().rotateBy(retStates[i].angle);
      if (flipHeading(deltaRotation)) {
        retStates[i].angle = retStates[i].angle.rotateBy(Rotation2d.fromRadians(Math.PI));
        retStates[i].speedMetersPerSecond *= -1.0;
      }
    }
    return new SwerveSetpoint(retSpeeds, retStates);
  }
}
