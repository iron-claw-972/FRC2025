package frc.robot.commands.gpm;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import frc.robot.constants.IntakeConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.outtake.Outtake;

public class OuttakeAlgae extends ConditionalCommand {
    public OuttakeAlgae(Outtake outtake, Intake intake){
        super(
            new OuttakeAlgaeArm(outtake),
            new OuttakeAlgaeIntake(intake),
            ()-> intake.isAtSetpoint(IntakeConstants.STOW_SETPOINT)
        );
    }
}
