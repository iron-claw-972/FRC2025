package frc.robot.commands.gpm;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.StorageIndexConstants;
import frc.robot.subsystems.gpm.StorageIndex;

public class IndexerFeed extends Command {

    private final StorageIndex m_index;
    private final Timer timer = new Timer();

    public IndexerFeed(StorageIndex index){
        this.m_index = index;
//        addRequirements(index);
    }

	@Override
    public void initialize (){
        m_index.ejectIntoShooter();
        timer.restart();
    }

	@Override
    public void execute(){

    }

	@Override
    public boolean isFinished(){

        return timer.hasElapsed(StorageIndexConstants.ejectShootTimeout);
        
    }

	@Override
    public void end(boolean interupted){

        m_index.stopIndex();

    }
    
}
