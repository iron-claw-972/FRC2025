package frc.robot.commands;

import frc.robot.subsystems.gpm.Arm;
import edu.wpi.first.wpilibj2.command.Command;

//for use with IntakeNote, OuttakeAmp, and Shoot commands
public class ArmToPos extends Command {

  private final Arm m_arm;
  private final double m_setpoint;

  public ArmToPos(Arm arm, double setpoint) {
    m_arm = arm;
    m_setpoint = setpoint;
    addRequirements(m_arm);
  }

  public void initialize() {
    m_arm.setAngle(m_setpoint);
  }

  public boolean isFinished() {
    return m_arm.atSetpoint();
  }
}
