package frc.robot.util;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.IntegerLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;

import java.time.Duration;
import java.util.ArrayList;
import java.util.function.Supplier;

/**
 * Utilty class for logging data to the DataLogManager.
 * View logs using MechanicalAdvantage's advantage scope (<a href="https://github.com/Mechanical-Advantage/AdvantageScope">...</a>)
 */
public class LogManager {

  public static DataLog DATA_LOG = DataLogManager.getLog();

  private static final ArrayList<Log<?>> logs = new ArrayList<>();

  public static <T> void add(Log<T> log) {
    logs.add(log);
  }

  public static <T> void add(String name, T value) {
    add(name, ()->value);
  }
  public static <T> void add(String name, Supplier<T> value) {
    add(new Log<>(name, value));
  }

  public static <T> void add(String name, Supplier<T> value, Duration duration) {
    add(new Log<>(name, value, duration));
  }

  public static void update() {
    // if (!Constants.DO_LOGGING) return;
    logs.forEach(Log::update);
  }

  /**
   * Records the metadata supplied by gversion (https://github.com/lessthanoptimal/gversion-plugin) in BuildData.java.
   */
  public static void recordMetadata() {
    new StringLogEntry(DATA_LOG, "BuildData/Maven Group").append(BuildData.MAVEN_GROUP);
    new StringLogEntry(DATA_LOG, "BuildData/Maven Name").append(BuildData.MAVEN_NAME); // The name of the repository
    new StringLogEntry(DATA_LOG, "BuildData/Version").append(BuildData.VERSION);
    new IntegerLogEntry(DATA_LOG, "BuildData/Git Revision").append(BuildData.GIT_REVISION);
    new StringLogEntry(DATA_LOG, "BuildData/Git SHA").append(BuildData.GIT_SHA); // The SHA code for the latest commit
    new StringLogEntry(DATA_LOG, "BuildData/Git date").append(BuildData.GIT_DATE);
    new StringLogEntry(DATA_LOG, "BuildData/Git Branch").append(BuildData.GIT_BRANCH); // The branch name
    new StringLogEntry(DATA_LOG, "BuildData/Build Date").append(BuildData.BUILD_DATE);
    new IntegerLogEntry(DATA_LOG, "BuildData/Build Unix Time").append(BuildData.BUILD_UNIX_TIME);
    new IntegerLogEntry(DATA_LOG, "BuildData/Dirty").append(BuildData.DIRTY);
  }
}