package frc.robot.util;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.IntegerLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import frc.robot.constants.Constants;

import java.time.Duration;
import java.util.ArrayList;
import java.util.function.Supplier;

import dev.doglog.DogLog;

/**
 * Utilty class for logging data to the DataLogManager.
 * View logs using MechanicalAdvantage's advantage scope (<a href="https://github.com/Mechanical-Advantage/AdvantageScope">...</a>)
 */
public class LogManager extends DogLog {

  public static DataLog DATA_LOG = DataLogManager.getLog();

  private static final ArrayList<Log<?>> logs = new ArrayList<>();

  public static <T> void add(Log<T> log) {
    logs.add(log);
  }

  public static <T> void add(String name, T value) { 
    Class<?> type = value.getClass(); // Get class

    if (type == Integer.class) { // Check the type
      log(name, (Integer) value); // Since we know this is an integer, cast this to an integer
    }
    if (type == Double.class) { // Check the type
      log(name, (Double) value);
    }
    if (type == Boolean.class) { // Check the type
      log(name, (Boolean) value);
    }
    if (type == Long.class) { // Check the type
      log(name, (Long) value);
    }
  }


  public static <T> void log(String name, Supplier<T> value) {
    add(new Log<>(name, value));
  }

  /**
   * @deprecated Use log() instead
   * @param <T> Type of item being logged
   * @param name Name (key) of item being logged
   * @param value Supplier for value being logged 
   */
  public static <T> void add(String name, Supplier<T> value) {
    log(name, value);
  }

  public static <T> void add(String name, Supplier<T> value, Duration duration) {
    add(new Log<>(name, value, duration)); //todo
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