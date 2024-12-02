package frc.robot.subsystems.gpm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.StorageIndexConstants;

/**
 * The StorageIndex subsystem handles the control of the storage index
 * mechanism.
 */
public class StorageIndex extends SubsystemBase {

  private final CANSparkMax m_indexmotor;
  DigitalInput m_indexBeamBreak;

  /**
   * Constructs the StorageIndex subsystem, initializing the motor and beam break
   * sensor.
   */
  public StorageIndex() {
    m_indexmotor = new CANSparkMax(StorageIndexConstants.indexMotorID, MotorType.kBrushless);
    m_indexBeamBreak = new DigitalInput(StorageIndexConstants.indexBeamBreak);
    m_indexmotor.setInverted(false);

    m_indexmotor.setIdleMode(StorageIndexConstants.idleMode);


    // Additional setup, possibly related to CAN Frames, could be documented here.
  }

  public void close() {
    m_indexmotor.close();
    m_indexBeamBreak.close();
  }

  @Override
  public void periodic() {
  }

  /**
   * Stores a note in the indexer. Runs the index until a note is detected.
   */
  public void storeNote() {
    if (!hasNote()) {
      runIndex();
    }
  }

  /**
   * Runs the storage index mechanism at the specified speed.
   *
   * @param speed The speed at which to run the storage index mechanism.
   *              TODO: Consider whether to run the motor at a fixed RPM instead
   *              of a percentage of power.
   */
  public void runIndex(double speed) {
    m_indexmotor.set(speed);
  }

  /**
   * Runs the storage index mechanism at the default intake speed.
   * TODO: Consider whether to run the motor at a fixed RPM instead of a
   * percentage of power.
   */
  public void runIndex() {
    m_indexmotor.set(StorageIndexConstants.intakeSpeed);
  }

  /**
   * Stops the storage index mechanism.
   */
  public void stopIndex() {
    m_indexmotor.set(0);
  }

  /**
   * Ejects notes backward at the specified speed, if a note is present.
   *
   */
  public void ejectBack() {
    this.runIndex((-1.0) * StorageIndexConstants.intakeSpeed);
  }

  /**
   * Ejects notes forward at the specified speed, if a note is present.
   *
   */
  public void ejectAmpFront() {
    this.runIndex(StorageIndexConstants.ejectAmpFrontSpeed);
  }

  public void ejectTrap() {
    this.runIndex(StorageIndexConstants.ejectTrapSpeed);
  }

  public void ejectIntoShooter(){
    runIndex(StorageIndexConstants.ejectShootSpeed);
  }

  /**
   * Checks if a note is present using the beam break sensor.
   *
   * @return True if a note is present, false otherwise.
   */
  public boolean hasNote() {
    return !m_indexBeamBreak.get(); // Inverted as beambreak sensor returns true when beam not broken and false when
                                    // beam is broken
  }

}
