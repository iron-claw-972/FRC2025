package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
    @AutoLog
    public class IndexerIOInputs{
        public double velocity = 0.0;
        public boolean isIndexerClear = true;
        public int sensorDistance = 0;
    }
}
