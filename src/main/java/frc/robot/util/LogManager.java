package frc.robot.util;

import java.util.ArrayList;
import java.util.function.Supplier;

import dev.doglog.DogLog;
import frc.robot.constants.Constants;

/**
 * Utilty class for logging data to the DataLogManager.
 * View logs using MechanicalAdvantage's advantage scope (<a href="https://github.com/Mechanical-Advantage/AdvantageScope">...</a>)
 */
public class LogManager extends DogLog {

  static {
    if (Constants.LOG_LEVEL == LogLevel.NONE)
      setEnabled(false);
  }
  
  
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
   * Log a Log object periodically
   * @param <T> Type of item being logged
   * @param log The Log object to log
   * @param minLogLevel Minimum log level needed to log this item @see {@link LogLevel}
   */
  public static <T> void log(Log<T> log, LogLevel minLogLevel) {
    if (Constants.LOG_LEVEL.value >= minLogLevel.value && Constants.LOG_LEVEL.value != 0)
      logs.add(log);
    else
      return;
  }
 
    /**
   * Log a supplier and log if it is in the specified range
   * @param <T> Type of item being logged
   * @param name Name (key) of item being logged
   * @param value Current int value being logged 
   * @param min The minimum that the value can be
   * @param max The maximum that the value can be
   */ 
  public static void log(String name, int value, int min, int max) {
    log(name, value);
    
    if (value < min || value > max) {
      logFault(name + " is out of specified range!");
    }
  }

  /**
   * Log a supplier and log if it is in the specified range
   * @param <T> Type of item being logged
   * @param name Name (key) of item being logged
   * @param value Current double value being logged 
   * @param min The minimum that the value can be
   * @param max The maximum that the value can be
   */
  public static void log(String name, double value, double min, double max) {
    log(name, value);
    
    if (value < min || value > max) {
      logFault(name + " is out of specified range!");
    }
  }

  /**
   * Log a supplier and log if it is in the specified range
   * @param <T> Type of item being logged
   * @param name Name (key) of item being logged
   * @param value Current long value being logged 
   * @param min The minimum that the value can be
   * @param max The maximum that the value can be
   */
  public static void log(String name, long value, long min, long max) {
    log(name, value);
    
    if (value < min || value > max) {
      logFault(name + " is out of specified range!");
    }
  }

    /**
   * Log a supplier and log if it is in the specified range
   * @param <T> Type of item being logged
   * @param name Name (key) of item being logged
   * @param value Supplier for value being logged 
   * @param min The minimum that the value can be
   * @param max The maximum that the value can be
   */
  public static void log(String name, float value, float min, float max) {
    log(name, value);
    
    if (value < min || value > max) {
      logFault(name + " is out of specified range!");
    }
  }
  
    /**
   * Log a supplier every 20ms
   * @param <T> Type of item being logged
   * @param name Name (key) of item being logged
   * @param value Supplier for value being logged 
   */
  public static <T> void logSupplier(String name, Supplier<T> value) {
    log(new Log<>(name, value));
  }

    /**
   * Log a supplier every 20ms with a minimum and a maxium, and if the value is outside of the min or max, it will log a fault.
   * @param <T> Type of item being logged
   * @param name Name (key) of item being logged
   * @param value Supplier for value being logged 
   * @param min The minimum that the value can be
   * @param min The maximum that the value can be
   */
  public static <T> void logSupplier(String name, Supplier<T> value, T min, T max) {
    log(new Log<>(name, value, min, max));
  }

  /**
   * Log a supplier periodically
   * @param <T> Type of item being logged
   * @param name Name (key) of item being logged
   * @param value Supplier for value being logged 
   * @param updateDelay The amount of time, in milliseconds, between logs
   */
  public static <T> void logSupplier(String name, Supplier<T> value, int updateDelay) {
    log(new Log<>(name, value, updateDelay));
  }

    /**
   * Log a supplier periodically with a minimum and a maxium, and if the value is outside of the min or max, it will log a fault. 
   * @param <T> Type of item being logged
   * @param name Name (key) of item being logged
   * @param value Supplier for value being logged 
   * @param updateDelay The amount of time, in milliseconds, between logs
   * @param min The minimum that the value can be
   * @param min The maximum that the value can be

   */
  public static <T> void logSupplier(String name, Supplier<T> value, int updateDelay, T min, T max) {
    log(new Log<>(name, value, updateDelay, min, max));
  }

  
  /**
   * Log a supplier every 20ms
   * @param <T> Type of item being logged
   * @param name Name (key) of item being logged
   * @param value Supplier for value being logged
   * @param minLogLevel Minimum log level needed to log this item @see {@link LogLevel} 
   */
  public static <T> void logSupplier(String name, Supplier<T> value, LogLevel minLogLevel) {
    if (Constants.LOG_LEVEL.value >= minLogLevel.value && Constants.LOG_LEVEL.value != 0)
      log(new Log<>(name, value));
    else
      return;
    
  }

    /**
   * Log a supplier every 20ms with a minimum and a maxium, and if the value is outside of the min or max, it will log a fault.
   * @param <T> Type of item being logged
   * @param name Name (key) of item being logged
   * @param value Supplier for value being logged 
   * @param min The minimum that the value can be
   * @param min The maximum that the value can be
   * @param minLogLevel Minimum log level needed to log this item @see {@link LogLevel} 
   */
  public static <T> void logSupplier(String name, Supplier<T> value, T min, T max, LogLevel minLogLevel) {
    if (Constants.LOG_LEVEL.value >= minLogLevel.value && Constants.LOG_LEVEL.value != 0)
      log(new Log<>(name, value, min, max));
    else
      return;
  }


  /**
   * Log a supplier periodically
   * @param <T> Type of item being logged
   * @param name Name (key) of item being logged
   * @param value Supplier for value being logged 
   * @param updateDelay The amount of time, in milliseconds, between logs
   * @param minLogLevel Minimum log level needed to log this item @see {@link LogLevel} 
   */
  public static <T> void logSupplier(String name, Supplier<T> value, int updateDelay, LogLevel minLogLevel) {
    if (Constants.LOG_LEVEL.value >= minLogLevel.value && Constants.LOG_LEVEL.value != 0)
      log(new Log<>(name, value, updateDelay));
    else
      return;
  }

    /**
   * Log a supplier periodically with a minimum and a maxium, and if the value is outside of the min or max, it will log a fault. 
   * @param <T> Type of item being logged
   * @param name Name (key) of item being logged
   * @param value Supplier for value being logged 
   * @param updateDelay The amount of time, in milliseconds, between logs
   * @param min The minimum that the value can be
   * @param min The maximum that the value can be
   * @param minLogLevel Minimum log level needed to log this item @see {@link LogLevel} 

   */
  public static <T> void logSupplier(String name, Supplier<T> value, int updateDelay, T min, T max, LogLevel minLogLevel) {
    if (Constants.LOG_LEVEL.value >= minLogLevel.value && Constants.LOG_LEVEL.value != 0)
      log(new Log<>(name, value, updateDelay, min, max));
    else
      return;
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

  public enum LogLevel {
    DEBUG(3),
    INFO(2)
    COMP(1),
    NONE(0);

    private int value;

    private LogLevel(int level) {
      this.value = level;
    }

    public int getValue() {
      return value;
    }


  }
}