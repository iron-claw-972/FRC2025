package frc.robot.util;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.constants.Constants;
import java.io.IOError;
import java.io.IOException;

/**
 * Utility class for easy creation of motor controllers.
 */
public class MotorFactory {

    private static final int SPARK_MAX_DEFAULT_CURRENT_LIMIT = 60;

    ///////////////////////////////////////////////////////////////////////////////////////////////
    // SPARK MAX
    ///////////////////////////////////////////////////////////////////////////////////////////////

    /**
     * Create a CANSparkMax with current limiting enabled
     *
     * @param id         the ID of the Spark MAX
     * @param motortype  the type of motor the Spark MAX is connected to
     * @param stallLimit the current limit to set at stall
     * @return a fully configured CANSparkMAX
     */
    public static CANSparkMax createSparkMAX(int id, MotorType motortype, int stallLimit) {
        CANSparkMax sparkMAX = new CANSparkMax(id, motortype);
        sparkMAX.restoreFactoryDefaults();
        sparkMAX.enableVoltageCompensation(Constants.ROBOT_VOLTAGE);
        sparkMAX.setSmartCurrentLimit(stallLimit);
        sparkMAX.setIdleMode(IdleMode.kBrake);

        sparkMAX.burnFlash();
        return sparkMAX;
    }

    /**
     * Create a CANSparkMax with default current limiting enabled
     *
     * @param id        the ID of the Spark MAX
     * @param motortype the type of motor the Spark MAX is connected to
     * @return a fully configured CANSparkMAX
     */
    public static CANSparkMax createSparkMAXDefault(int id, MotorType motortype) {
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

        if (RobotBase.isReal() && talon.getVersion().getValue() != Constants.FIRMWARE_VERSION) {
            String errorMessage = "TalonFX " + id + " firmware incorrect. Has " + talon.getVersion().getValue()
                                  + ", currently FalconConstants.java requires: " + Constants.FIRMWARE_VERSION;
            if (Constants.BREAK_ON_WRONG_FIRMWARE) {
                DriverStation.reportError(errorMessage, true);
                throw new IOError(new IOException(errorMessage));
            } else {
                DriverStation.reportWarning(errorMessage + ", ignoring due to user specification.", false);
            }
        }

        TalonFXConfiguration config = new TalonFXConfiguration();

        // See explanations for Supply and Stator limiting in FalconConstants.java
        config.CurrentLimits = new CurrentLimitsConfigs().withStatorCurrentLimitEnable(StatorLimitEnable).withStatorCurrentLimit(StatorCurrentLimit).
            withSupplyCurrentLimitEnable(SupplyLimitEnable).withSupplyCurrentLimit(SupplyCurrentLimit).
            withSupplyCurrentThreshold(SupplyTriggerThreshold).withSupplyTimeThreshold(SupplyTriggerDuration);

        // TODO: Previous variable doesn't exist, might or might not be correct
        config.Voltage = new VoltageConfigs().withPeakForwardVoltage(Constants.ROBOT_VOLTAGE);

        // TODO: I can't find where these settings are
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
        return createTalonFXFull(id, CANBus, Constants.STATOR_LIMIT_ENABLE, Constants.STATOR_CURRENT_LIMIT,
                                 Constants.STATOR_TRIGGER_THRESHOLD, Constants.STATOR_TRIGGER_DURATION,
                                 Constants.SUPPLY_LIMIT_ENABLE, Constants.SUPPLY_CURRENT_LIMIT,
                                 Constants.SUPPLY_TRIGGER_THRESHOLD, Constants.SUPPLY_TRIGGER_DURATION);
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
        return createTalonFXFull(id, CANBus, Constants.STATOR_LIMIT_ENABLE, Constants.STATOR_CURRENT_LIMIT,
                                 Constants.STATOR_TRIGGER_THRESHOLD, Constants.STATOR_TRIGGER_DURATION, true, currentLimit,
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
                                 Constants.SUPPLY_LIMIT_ENABLE, Constants.SUPPLY_CURRENT_LIMIT,
                                 Constants.SUPPLY_TRIGGER_THRESHOLD, Constants.SUPPLY_TRIGGER_DURATION);
    }
}