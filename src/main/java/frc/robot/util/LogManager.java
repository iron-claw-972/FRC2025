package frc.robot.util;

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


  private static final ArrayList<Log<?>> logs = new ArrayList<>();

  public static <T> void log(Log<T> log) {
    logs.add(log);
  }

  public static <T> void log(String name, T value) { 
    Class<?> type = value.getClass(); // Get class

    if (isInteger(type)) { // Check the type
      log(name, (Integer) value); // Since we know this is an integer, cast this to an integer
    } else if (isDouble(type)) { // Check the type
      log(name, (Double) value);
    } else if (isLong(type)) {
      log(name, (Long) value);
    } else if (isBoolean(type)) { // Check the type
      log(name, (Boolean) value);
    } else if (isIntegerArray(type)) { // Check the type
      log(name, (Integer[]) value);
    } else if (isDoubleArray(type)) {
      log(name, (Double[]) value);
    } else {
      throw new IllegalArgumentException("Unsupported Log Type: " + type);
    }
  }


  public static <T> void log(String name, Supplier<T> value) {
    log(new Log<>(name, value));
  }

  public static <T> void log(String name, Supplier<T> value, Duration duration) {
    log(new Log<>(name, value, duration)); //todo
  }

  

/*************** Old Add Methods **************/
  
  public static <T> void add(Log<T> log) {
    log(log);
  }

  /**
   * @deprecated Use {@link frc.robot.util.LogManager#log(String name, T value)} instead
   * @param <T> Type of item being logged
   * @param name Name (key) of item being logged
   * @param value Supplier for value being logged 
   */
   public static <T> void add(String name, T value) {
    log(name, value);
  }
   
   public static <T> void add(String name, Supplier<T> value) {
    log(name, value);
  }

  public static <T> void add(String name, Supplier<T> value, Duration duration) {
    log(name, value, duration); 
  }

  

  public static void update() {
    // if (!Constants.DO_LOGGING) return;
    logs.forEach(Log::update);
  }

  /**
   * Records the metadata supplied by gversion (https://github.com/lessthanoptimal/gversion-plugin) in BuildData.java.
   */
  public static void recordMetadata() {
    log("BuildData/Maven Group",BuildData.MAVEN_GROUP);
    log("BuildData/Maven Name", BuildData.MAVEN_NAME); // The name of the repository
    log("BuildData/Version",BuildData.VERSION);
    log("BuildData/Git Revision",BuildData.GIT_REVISION);
    log("BuildData/Git SHA",BuildData.GIT_SHA); // The SHA code for the latest commit
    log("BuildData/Git date",BuildData.GIT_DATE);
    log("BuildData/Git Branch",BuildData.GIT_BRANCH); // The branch name
    log("BuildData/Build Date",BuildData.BUILD_DATE);
    log("BuildData/Build Unix Time",BuildData.BUILD_UNIX_TIME);
    log("BuildData/Dirty", BuildData.DIRTY);
  }


  private static boolean isInteger(Class<?> cls) {
    return cls == Integer.class;
  }

  private static boolean isDouble(Class<?> cls) {
      return cls == Double.class;
  }

  private static boolean isLong(Class<?> cls) {
    return cls == Long.class;
}
  
  private static boolean isBoolean(Class<?> cls) {
    return cls == Boolean.class;
  }

  private static boolean isIntegerArray(Class<?> cls) {
      return cls == Integer[].class;
  }

  private static boolean isDoubleArray(Class<?> cls) {
      return cls == Double[].class;
  }
}