package frc.robot.util;

import edu.wpi.first.util.datalog.*;

import java.time.Duration;
import java.util.Arrays;
import java.util.function.Supplier;

import dev.doglog.DogLog;

public class Log<T> {
    private final String name;
    private final Supplier<T> supplier;
    private T value;
    private final Duration delay;
    private long lastUpdate = 0;

    // private final DataLogEntry logEntry;


    public Log(String name, Supplier<T> supplier, Duration delay) {
        this.name = name;
        this.supplier = supplier;
        this.delay = delay;

        this.value = supplier.get();
        // if (isInteger()) {
            
            // logEntry = new IntegerLogEntry(LogManager.DATA_LOG, name);
        // } else if (isDouble()) {
        //     // logEntry = new DoubleLogEntry(LogManager.DATA_LOG, name);
        // } else if (isIntegerArray()) {
        //     // logEntry = new IntegerArrayLogEntry(LogManager.DATA_LOG, name);
        // } else if (isDoubleArray()) {
        //     // logEntry = new DoubleArrayLogEntry(LogManager.DATA_LOG, name);
        // } else {
        //     throw new IllegalArgumentException("Unsupported log type: " + value.getClass());
        // }
    }

    public Log(String name, Supplier<T> value) {
        this(name, value, Duration.ofMillis(20));
    }

    public void update() {
        if (System.currentTimeMillis() - lastUpdate > delay.toMillis()) {
            value = supplier.get();
            lastUpdate = System.currentTimeMillis();

            LogManager.log(name, value);

        //     if (isInteger()) {
        //         LogManager.log(name, (Integer) value); 
        //     } else if (isDouble()) {
        //         LogManager.log(name, (Double) value);
        //     } else if (isIntegerArray()) {
        //         long[] array = Arrays.stream((Integer[]) value).mapToLong(Integer::longValue).toArray();
        //         LogManager.log(name, array);
        //     } else if (isDoubleArray()) {
        //         var array = Arrays.stream((Double[]) value).mapToDouble(Double::doubleValue).toArray();
        //         LogManager.log(name, array); 
        //     }
        }
    }

    public String getName() {
        return name;
    }

    public Supplier<T> getSupplier() {
        return supplier;
    }

    public T getValue() {
        return value;
    }

    public Duration getDelay() {
        return delay;
    }
}