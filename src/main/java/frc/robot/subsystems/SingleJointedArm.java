package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.swerve.DriveConstants;

public class SingleJointedArm extends SubsystemBase {
    // TODO: Use that sim object to update the sim state using .addRotorPosition()
    // with the sim velocity * time or .setRawRotorPosition() with the position.

    // TODO: Tune the PID

    // TODO: Add the sensor once they figure out what type it will be

    // TODO: Maybe simulate that sensor and make it activate 1 second after the
    // intake starts (low priority)

    // TODO: put in proper id

 private final TrapezoidProfile m_profile =
      new TrapezoidProfile(
          new TrapezoidProfile.Constraints(
              Units.degreesToRadians(45),
              Units.degreesToRadians(90))); // Max arm speed and acceleration.
  private TrapezoidProfile.State m_lastProfiledReference = new TrapezoidProfile.State();

  // The plant holds a state-space model of our arm. This system has the following properties:
  //
  // States: [position, velocity], in radians and radians per second.
  // Inputs (what we can "put in"): [voltage], in volts.
  // Outputs (what we can measure): [position], in radians.
  private final LinearSystem<N2, N1, N2> m_armPlant =
      LinearSystemId.createSingleJointedArmSystem(DCMotor.getNEO(1), IntakeConstants.MOI, IntakeConstants.gearing);

  // The observer fuses our encoder data and voltage inputs to reject noise.
  @SuppressWarnings("unchecked")
  private final KalmanFilter<N2, N1, N1> m_observer =
      new KalmanFilter<>(
          Nat.N2(),
          Nat.N1(),
          (LinearSystem<N2, N1, N1>) m_armPlant.slice(0),
          VecBuilder.fill(0.015, 0.17), // How accurate we
          // think our model is, in radians and radians/sec
          VecBuilder.fill(0.01), // How accurate we think our encoder position
          // data is. In this case we very highly trust our encoder position reading.
          0.020);

  // A LQR uses feedback to create voltage commands.
  @SuppressWarnings("unchecked")
  private final LinearQuadraticRegulator<N2, N1, N1> m_controller =
      new LinearQuadraticRegulator<>(
          (LinearSystem<N2, N1, N1>) m_armPlant.slice(0),
          VecBuilder.fill(Units.degreesToRadians(1.0), Units.degreesToRadians(10.0)), // qelms.
          // Position and velocity error tolerances, in radians and radians per second. Decrease
          // this
          // to more heavily penalize state excursion, or make the controller behave more
          // aggressively. In this example we weight position much more highly than velocity, but
          // this
          // can be tuned to balance the two.
          VecBuilder.fill(12.0), // relms. Control effort (voltage) tolerance. Decrease this to more
          // heavily penalize control effort, or make the controller less aggressive. 12 is a good
          // starting point because that is the (approximate) maximum voltage of a battery.
          0.020); // Nominal time between loops. 0.020 for TimedRobot, but can be

  // lower if using notifiers.

  // The state-space loop combines a controller, observer, feedforward and plant for easy control.
  @SuppressWarnings("unchecked")
  private final LinearSystemLoop<N2, N1, N1> m_loop =
      new LinearSystemLoop<>(
          (LinearSystem<N2, N1, N1>) m_armPlant.slice(0), m_controller, m_observer, 12.0, 0.020);

    SparkMax motor = new SparkMax(0, MotorType.kBrushless);
    RelativeEncoder encoder = motor.getAlternateEncoder();
    double setpoint = IntakeConstants.startingAngle;


    public SingleJointedArm() {
        
        encoder.setPosition(IntakeConstants.startingAngle);
        // Reset our loop to make sure it's in a known state.
    m_loop.reset(VecBuilder.fill(getPosition(), getVelocity()));
    
    // Reset our last reference to the current state.
    m_lastProfiledReference =
        new TrapezoidProfile.State(getPosition(), getVelocity());
        if (RobotBase.isSimulation()) {
            // TODO: Add more simulation-specific behavior if needed

        }
        

        
    }

    /**
     * publishes stuff to smartdashboard
     */
    @Override
    public void periodic() {
       

    // Sets the target position of our arm. This is similar to setting the setpoint of a
    // PID controller.
    TrapezoidProfile.State goal = new State(setpoint,0);
    // Step our TrapezoidalProfile forward 20ms and set it as our next reference
    m_lastProfiledReference = m_profile.calculate(0.020, m_lastProfiledReference, goal);
    m_loop.setNextR(m_lastProfiledReference.position, m_lastProfiledReference.velocity);
    // Correct our Kalman filter's state vector estimate with encoder data.
    m_loop.correct(VecBuilder.fill(getPosition()));

    // Update our LQR to generate new voltage commands and use the voltages to predict the next
    // state with out Kalman filter.
    m_loop.predict(0.020);

    // Send the new calculated voltage to the motors.
    // voltage = duty cycle * battery voltage, so
    // duty cycle = voltage / battery voltage
    double nextVoltage = m_loop.getU(0);
    motor.setVoltage(nextVoltage);
        
    }

    @Override
    public void simulationPeriodic() {
        // TODO: Add simulation-specific periodic logic if needed
        
    }

    public double getPosition(){
       return encoder.getPosition()*2*Math.PI/IntakeConstants.gearing;
    }
    
    public double getVelocity(){
        return encoder.getVelocity()*2*Math.PI/60/IntakeConstants.gearing;
     }

     public void setSetpoint(double setpoint){
        this.setpoint = MathUtil.clamp(setpoint, IntakeConstants.minAngle, IntakeConstants.maxAngle);
     }
}
