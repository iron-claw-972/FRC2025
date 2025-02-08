package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N20;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.IdConstants;
import frc.robot.constants.IntakeConstants;

public class SingleJointedArm extends SubsystemBase {
    private final TrapezoidProfile m_profile =
        new TrapezoidProfile(
            new TrapezoidProfile.Constraints(
                Units.degreesToRadians(90)*10,
                Units.degreesToRadians(90)*10)); // Max arm speed and acceleration.
    private TrapezoidProfile.State m_lastProfiledReference = new TrapezoidProfile.State();

    // The plant holds a state-space model of our arm. This system has the following properties:
    //
    // States: [position, velocity], in radians and radians per second.
    // Inputs (what we can "put in"): [voltage], in volts.
    // Outputs (what we can measure): [position], in radians.
    private final LinearSystem<N2, N1, N2> m_armPlant =
        LinearSystemId.createSingleJointedArmSystem(IntakeConstants.MOTOR, IntakeConstants.MOI, IntakeConstants.GEARING);

    // The observer fuses our encoder data and voltage inputs to reject noise.
    @SuppressWarnings("unchecked")
    private final KalmanFilter<N2, N1, N2> m_observer =
        new KalmanFilter<>(
            Nat.N2(),
            Nat.N2(),
            (LinearSystem<N2, N1, N2>) m_armPlant,
            VecBuilder.fill(0.15, 0.17), // How accurate we
            // think our model is, in radians and radians/sec
            VecBuilder.fill(0.0001,0.0001), // How accurate we think our encoder position
            // data is. In this case we very highly trust our encoder position reading.
            0.020);

    // A LQR uses feedback to create voltage commands.
    @SuppressWarnings("unchecked")
    private final LinearQuadraticRegulator<N2, N1, N2> m_controller =
        new LinearQuadraticRegulator<>(
            (LinearSystem<N2, N1, N2>) m_armPlant,
            VecBuilder.fill(Units.degreesToRadians(0.1), Units.degreesToRadians(1.0)), // qelms.
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
    private final LinearSystemLoop<N2, N1, N2> m_loop =
        new LinearSystemLoop<>(
            (LinearSystem<N2, N1, N2>) m_armPlant, m_controller, m_observer, 12.0, 0.020);

    private SparkMax motor = new SparkMax(IdConstants.INTAKE_PIVOT, MotorType.kBrushless);
    private RelativeEncoder encoder = motor.getEncoder();
    private double setpoint = IntakeConstants.STARTING_ANGLE;

    // Sim stuff
    private SingleJointedArmSim sim;
    private Mechanism2d mechanism;
    private MechanismLigament2d ligament;
    private double voltage = 0;
    private double time = 0;
    private SparkRelativeEncoderSim encoderSim;

    public SingleJointedArm() {
        encoder.setPosition(Units.radiansToRotations(IntakeConstants.STARTING_ANGLE));
        // Reset our loop to make sure it's in a known state.
        m_loop.reset(VecBuilder.fill(getPosition(), getVelocity()));
    
        // Reset our last reference to the current state.
        m_lastProfiledReference =
            new TrapezoidProfile.State(getPosition(), getVelocity());
        if (RobotBase.isSimulation()) {
            sim = new SingleJointedArmSim(
                m_armPlant,
                IntakeConstants.MOTOR,
                IntakeConstants.GEARING,
                IntakeConstants.ARM_LENGTH,
                IntakeConstants.MIN_ANGLE,
                IntakeConstants.MAX_ANGLE,
                true,
                IntakeConstants.STARTING_ANGLE
            );
            mechanism = new Mechanism2d(10, 10);
            ligament = mechanism.getRoot("root", 5, 5).append(
                new MechanismLigament2d("Intake pivot", 4, Units.radiansToDegrees(IntakeConstants.STARTING_ANGLE))
            );
            encoderSim = new SparkRelativeEncoderSim(motor);
            encoderSim.setPosition(Units.radiansToRotations(IntakeConstants.STARTING_ANGLE)*IntakeConstants.GEARING);
            SmartDashboard.putData("Intake pivot", mechanism);
            SmartDashboard.putNumber("setpoint", 0);
        }
        setpoint = Math.PI/4;
    }

    /**
     * publishes stuff to smartdashboard
     */
    @Override
    public void periodic() {
        // Sets the target position of our arm. This is similar to setting the setpoint of a
        // PID controller.
    
        TrapezoidProfile.State goal = new State(SmartDashboard.getNumber("setpoint", 0),0);
        // Step our TrapezoidalProfile forward 20ms and set it as our next reference
        m_lastProfiledReference = m_profile.calculate(0.020, m_lastProfiledReference, goal);
        m_loop.setNextR(m_lastProfiledReference.position, m_lastProfiledReference.velocity);
        // Correct our Kalman filter's state vector estimate with encoder data.
        m_loop.correct(VecBuilder.fill(getPosition(), getVelocity()));
        

        // Update our LQR to generate new voltage commands and use the voltages to predict the next
        // state with out Kalman filter.
        m_loop.predict(0.020);

        // Send the new calculated voltage to the motors.
        // voltage = duty cycle * battery voltage, so
        // duty cycle = voltage / battery voltage
        // Stored for sim
        voltage = m_loop.getU(0);
        
        motor.setVoltage(voltage);
    }
    
    @Override
    public void simulationPeriodic() {
        sim.setInputVoltage(voltage);
        // For the first frame, set the previous time to 20ms before the current time
        if(time < 0.00001){
            time = Timer.getFPGATimestamp() - Constants.LOOP_TIME;
        }
        // More accurately simulate things like loop overruns and other random timing errors
        double currentTime = Timer.getFPGATimestamp();
        sim.update(currentTime - time);
        time = currentTime;
        encoderSim.setPosition(Units.radiansToRotations(sim.getAngleRads())*IntakeConstants.GEARING);
        encoderSim.setVelocity(Units.radiansPerSecondToRotationsPerMinute(sim.getVelocityRadPerSec())*IntakeConstants.GEARING);
        ligament.setAngle(Units.radiansToDegrees(sim.getAngleRads()));
        System.out.println("voltage "+voltage);
        System.out.println("position "+sim.getAngleRads());
    }

    public double getPosition(){
       return Units.rotationsToRadians(encoder.getPosition())/IntakeConstants.GEARING;
    }
    
    public double getVelocity(){
        return Units.rotationsPerMinuteToRadiansPerSecond(encoder.getVelocity())/IntakeConstants.GEARING;
     }

     public void setSetpoint(double setpoint){
        this.setpoint = MathUtil.clamp(setpoint, IntakeConstants.MIN_ANGLE, IntakeConstants.MAX_ANGLE);
     }
}
