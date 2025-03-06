package frc.robot.commands.vision;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.gpm.IntakeCoral;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Outtake;
import frc.robot.util.DetectedObject;

public class AcquireGamePiece extends SequentialCommandGroup {
    /**
     * Intakes a game piece
     * 
     * @param gamePiece The supplier for the game piece to intake
     * @param drive The drivetrain
     * @param intake The intake
     * @param index The indexer
     * @param arm The arm
     */
    public AcquireGamePiece(Supplier<DetectedObject> gamePiece, Drivetrain drive, Intake intake, Indexer indexer, Elevator elevator, Outtake outtake){
        addCommands(new IntakeCoral(intake, indexer, elevator, outtake).deadlineWith(new DriveToCoral(gamePiece, drive)));
    }
}