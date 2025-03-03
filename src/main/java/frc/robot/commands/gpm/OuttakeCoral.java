package frc.robot.commands.gpm;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.constants.ElevatorConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Outtake;

public class OuttakeCoral extends SequentialCommandGroup {
    public OuttakeCoral(Outtake outtake, Elevator elevator){
        BooleanSupplier l4Supplier = ()-> elevator.getSetpoint() > ElevatorConstants.L3_SETPOINT + 0.001;
        addCommands(
            new ConditionalCommand(
                new ScoreL4(elevator, outtake),
                new OuttakeCoralBasic(outtake, l4Supplier),
                l4Supplier)
            //new InstantCommand(()->elevator.setSetpoint(ElevatorConstants.STOW_SETPOINT), elevator)
        );
    }
}
