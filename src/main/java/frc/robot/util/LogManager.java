package frc.robot.util;

import java.util.ArrayList;
import java.util.function.Supplier;

import dev.doglog.DogLog;

/**
 * Utilty class for logging data to the DataLogManager.
 * View logs using MechanicalAdvantage's advantage scope (<a href="https://github.com/Mechanical-Advantage/AdvantageScope">...</a>)
 */
public class LogManager extends DogLog {

  private static final ArrayList<Log<?>> logs = new ArrayList<>();

  /**
   * Log a Log object periodically
   * @param <T> Type of item being logged
   * @param log The Log object to log
   */
  public static <T> void log(Log<T> log) {
    logs.add(log);
  }
  
  /**
   * Log a supplier every 20ms
   * @param <T> Type of item being logged
   * @param name Name (key) of item being logged
   * @param value Supplier for value being logged 
   */
  public static <T> void log(String name, Supplier<T> value) {
    log(new Log<>(name, value));
  }
  
  /**
   * Log a supplier periodically
   * @param <T> Type of item being logged
   * @param name Name (key) of item being logged
   * @param value Supplier for value being logged 
   * @param updateDelay The amount of time, in milliseconds, between logs
   */
  public static <T> void log(String name, Supplier<T> value, int updateDelay) {
    log(new Log<>(name, value, updateDelay));
  }

  /**
   * Update logs
   */
  public static void update() {
    // if (!Constants.DO_LOGGING) return;
    logs.forEach(Log::update);
  }

  /**
   * Records the metadata supplied by gversion (https://github.com/lessthanoptimal/gversion-plugin) in BuildData.java.
   */
  public static void recordMetadata() {
    log("BuildData/Maven Group", BuildData.MAVEN_GROUP);
    log("BuildData/Maven Name", BuildData.MAVEN_NAME); // The name of the repository
    log("BuildData/Version", BuildData.VERSION);
    log("BuildData/Git Revision", BuildData.GIT_REVISION);
    log("BuildData/Git SHA", BuildData.GIT_SHA); // The SHA code for the latest commit
    log("BuildData/Git date", BuildData.GIT_DATE);
    log("BuildData/Git Branch", BuildData.GIT_BRANCH); // The branch name
    log("BuildData/Build Date", BuildData.BUILD_DATE);
    log("BuildData/Build Unix Time", BuildData.BUILD_UNIX_TIME);
    log("BuildData/Dirty", BuildData.DIRTY);
  }
}