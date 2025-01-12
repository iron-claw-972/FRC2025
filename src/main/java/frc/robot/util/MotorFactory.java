package frc.robot.util;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.constants.Constants;

/**
 * Utility class for easy creation of motor controllers.
 */
public class MotorFactory {

    private static final int SPARK_MAX_DEFAULT_CURRENT_LIMIT = 60;

    ///////////////////////////////////////////////////////////////////////////////////////////////
    // SPARK MAX
    ///////////////////////////////////////////////////////////////////////////////////////////////

    /**
     * Create a SparkMax with current limiting enabled
     *
     * @param id         the ID of the Spark MAX
     * @param motortype  the type of motor the Spark MAX is connected to
     * @param stallLimit the current limit to set at stall
     * @return a fully configured CANSparkMAX
     */
    public static SparkMax createSparkMAX(int id, MotorType motortype, int stallLimit) {
        SparkMax sparkMAX = new SparkMax(id, motortype);

        sparkMAX.configure(new SparkMaxConfig()
            .voltageCompensation(Constants.ROBOT_VOLTAGE)
            .smartCurrentLimit(stallLimit)
            .idleMode(IdleMode.kBrake),
            ResetMode.kResetSafeParameters,
            PersistMode.kNoPersistParameters
        );
        return sparkMAX;
    }

    /**
     * Create a SparkMax with default current limiting enabled
     *
     * @param id        the ID of the Spark MAX
     * @param motortype the type of motor the Spark MAX is connected to
     * @return a fully configured CANSparkMAX
     */
    public static SparkMax createSparkMAXDefault(int id, MotorType motortype) {
        return createSparkMAX(id, motortype, SPARK_MAX_DEFAULT_CURRENT_LIMIT);
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////
    // TALON FX (Falcon 500 and Kraken X60)
    ///////////////////////////////////////////////////////////////////////////////////////////////

    /**
     * Creates a TalonFX with all current limit options. If you would like to use
     * defaults it is recommended to use the other createTalonFX.. methods.
     *
     * @param id                     the CAN ID of the TalonFX
     * @param CANBus                 the CAN bus the TalonFX is on. If connected to the rio it is "rio".
     * @param StatorLimitEnable      whether to enable stator limiting
     * @param StatorCurrentLimit     the current, in amps, to return to after the
     *                               stator limit is triggered
     * @param StatorTriggerThreshold the threshold current to trigger the stator
     *                               limit
     * @param StatorTriggerDuration  the duration, in seconds, the current is above
     *                               the threshold before triggering
     * @param SupplyLimitEnable      whether to enable supply limiting
     * @param SupplyCurrentLimit     the current, in amps, to return to after the
     *                               supply limit is triggered
     * @param SupplyTriggerThreshold the threshold current to trigger the supply
     *                               limit
     * @param SupplyTriggerDuration  the duration, in seconds, the current is above
     *                               the threshold before triggering
     * @return A fully configured TalonFX
     */
    public static TalonFX createTalonFXFull(int id, String CANBus, boolean StatorLimitEnable,
                                                double StatorCurrentLimit,
                                                double StatorTriggerThreshold, double StatorTriggerDuration, boolean SupplyLimitEnable, double SupplyCurrentLimit,
                                                double SupplyTriggerThreshold, double SupplyTriggerDuration) {

        if (id == -1) {
            return null;
        }

        TalonFX talon = new TalonFX(id, CANBus);

        TalonFXConfiguration config = new TalonFXConfiguration();

        // See explanations for Supply and Stator limiting in FalconConstants.java
        config.CurrentLimits = new CurrentLimitsConfigs().withStatorCurrentLimitEnable(StatorLimitEnable).withStatorCurrentLimit(StatorCurrentLimit).
            withSupplyCurrentLimitEnable(SupplyLimitEnable).withSupplyCurrentLimit(SupplyCurrentLimit).
            withSupplyCurrentLowerLimit(SupplyTriggerThreshold).withSupplyCurrentLowerTime(SupplyTriggerDuration);

        config.Voltage = new VoltageConfigs().withPeakForwardVoltage(Constants.ROBOT_VOLTAGE);

        talon.getConfigurator().apply(config);
        talon.setNeutralMode(NeutralModeValue.Brake);
        

        return talon;
    }

    /**
     * Creates a TalonFX with all the default settings.
     *
     * @param id     the id of the motor
     * @param CANBus the CAN bus the TalonFX is on. If connected to the rio it is "rio".
     */
    public static TalonFX createTalonFX(int id, String CANBus) {
        return createTalonFXFull(id, CANBus, Constants.TALONFX_STATOR_LIMIT_ENABLE, Constants.TALONFX_STATOR_CURRENT_LIMIT,
                                 Constants.TALONFX_STATOR_TRIGGER_THRESHOLD, Constants.TALONFX_STATOR_TRIGGER_DURATION,
                                 Constants.TALONFX_SUPPLY_LIMIT_ENABLE, Constants.TALONFX_SUPPLY_CURRENT_LIMIT,
                                 Constants.TALONFX_SUPPLY_TRIGGER_THRESHOLD, Constants.TALONFX_SUPPLY_TRIGGER_DURATION);
    }

    /**
     * Creates a TalonFX with supply current limit options.
     * <p>
     * Supply current is current that's being drawn at the input bus voltage.
     * Supply limiting is useful for preventing breakers from tripping in the PDP.
     *
     * @param id               the CAN ID of the TalonFX
     * @param CANBus           the CAN bus the TalonFX is on. If connected to the rio it is "rio".
     * @param currentLimit     the current, in amps, to return to after the supply limit is triggered
     * @param triggerThreshold the threshold current to trigger the supply limit
     * @param triggerDuration  the duration, in seconds, the current is above the threshold before triggering
     */
    public static TalonFX createTalonFXSupplyLimit(int id, String CANBus, double currentLimit,
                                                       double triggerThreshold, double triggerDuration) {
        return createTalonFXFull(id, CANBus, Constants.TALONFX_STATOR_LIMIT_ENABLE, Constants.TALONFX_STATOR_CURRENT_LIMIT,
                                 Constants.TALONFX_STATOR_TRIGGER_THRESHOLD, Constants.TALONFX_STATOR_TRIGGER_DURATION, true, currentLimit,
                                 triggerThreshold, triggerDuration);
    }

    /**
     * Creates a TalonFX with stator current limit options.
     * <p>
     * Stator current is current that’s being drawn by the motor.
     * Stator limiting is useful for limiting acceleration/heat.
     *
     * @param id               the CAN ID of the TalonFX
     * @param CANBus           the CAN bus the TalonFX is on. If connected to the rio it is "rio".
     * @param currentLimit     the current, in amps, to return to after the stator limit is triggered
     * @param triggerThreshold the threshold current to trigger the stator limit
     * @param triggerDuration  the duration, in seconds, the current is above the threshold before triggering
     */
    public static TalonFX createTalonFXStatorLimit(int id, String CANBus, double currentLimit,
                                                       double triggerThreshold, double triggerDuration) {
        return createTalonFXFull(id, CANBus, true, currentLimit, triggerThreshold, triggerDuration,
                                 Constants.TALONFX_SUPPLY_LIMIT_ENABLE, Constants.TALONFX_SUPPLY_CURRENT_LIMIT,
                                 Constants.TALONFX_SUPPLY_TRIGGER_THRESHOLD, Constants.TALONFX_SUPPLY_TRIGGER_DURATION);
    }
}