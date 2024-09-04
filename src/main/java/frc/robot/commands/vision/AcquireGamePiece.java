package frc.robot.commands.vision;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.gpm.IntakeNote;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.gpm.Arm;
import frc.robot.subsystems.gpm.Intake;
import frc.robot.subsystems.gpm.StorageIndex;
import frc.robot.util.DetectedObject;

// public class AcquireGamePiece extends SequentialCommandGroup {
//     /**
//      * Intakes a game piece
//      * 
//      * @param gamePiece The supplier for the game piece to intake
//      * @param drive The drivetrain
//      * @param intake The intake
//      * @param index The indexer
//      * @param arm The arm
//      */
//     public AcquireGamePiece(Supplier<DetectedObject> gamePiece, Drivetrain drive, Intake intake, StorageIndex index, Arm arm){
//         addCommands(new IntakeNote(intake, index, arm).deadlineWith(new DriveToNote(gamePiece, drive)));
//     }
// }