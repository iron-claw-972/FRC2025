package frc.robot.util;

import java.util.Arrays;
import java.util.function.Supplier;

public class Log<T> {
    private final String name;
    private final Supplier<T> supplier;
    private T value;
    private final int delay;
    private long lastUpdate = 0;

    public Log(String name, Supplier<T> supplier, int delay) {
        this.name = name;
        this.supplier = supplier;
        this.delay = delay;

        this.value = supplier.get();
    }

    public Log(String name, Supplier<T> value) {
        this(name, value, 10); // Although this is 10, update will be called every 20ms
    }

    public void update() {
        long time = System.currentTimeMillis();
        if (time - lastUpdate > delay) {
            value = supplier.get();
            lastUpdate = time;

            if (isInteger()) {
                LogManager.log(name, (Integer) value); 
            } else if (isDouble()) {
                LogManager.log(name, (Double) value);
            } else if (isLong()) {
                LogManager.log(name, (Long) value);
            } else if (isBoolean()) {
                LogManager.log(name, (Boolean) value);
            } else if (isString()) {
                LogManager.log(name, (String) value);
            } else if (isIntegerArray()) {
                long[] array = Arrays.stream((Integer[]) value).mapToLong(Integer::longValue).toArray();
                LogManager.log(name, array);
            } else if (isIntArray()) {
                long[] array = Arrays.stream((int[]) value).mapToLong(i->i).toArray();
                LogManager.log(name, array);
            } else if (isDoubleArray()) {
                double[] array = Arrays.stream((Double[]) value).mapToDouble(Double::doubleValue).toArray();
                LogManager.log(name, array); 
            } else if (isDoubleArray2()) {
                LogManager.log(name, (double[]) value); 
            } else if (isLongArray()) {
                long[] array = Arrays.stream((Long[]) value).mapToLong(l->l).toArray();
                LogManager.log(name, array); 
            } else if (isLongArray2()) {
                LogManager.log(name, (long[]) value); 
            } else if (isBooleanArray()) {
                // For some reason, Java does not have BooleanStreams
                Boolean[] a = (Boolean[]) value;
                boolean[] array = new boolean[a.length];
                for(int i = 0; i < a.length; i++){
                    array[i] = a[i];
                }
                LogManager.log(name, array); 
            } else if (isBooleanArray2()) {
                LogManager.log(name, (long[]) value); 
            } else if (isStringArray()) {
                LogManager.log(name, (String[]) value); 
            }else{
                throw new IllegalArgumentException("Unsupported log type: " + value.getClass().getName());
            }
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

    public int getDelay() {
        return delay;
    }

    private boolean isInteger() {
        return value.getClass() == Integer.class;
    }

    private boolean isDouble() {
        return value.getClass() == Double.class;
    }

    private boolean isLong() {
        return value.getClass() == Long.class;
    }

    private boolean isBoolean() {
        return value.getClass() == Boolean.class;
    }

    private boolean isString() {
        return value.getClass() == String.class;
    }

    private boolean isIntegerArray() {
        return value.getClass() == Integer[].class;
    }

    private boolean isIntArray() {
        return value.getClass() == int[].class;
    }

    private boolean isDoubleArray() {
        return value.getClass() == Double[].class;
    }

    private boolean isDoubleArray2() {
        return value.getClass() == double[].class;
    }

    private boolean isLongArray() {
        return value.getClass() == Long[].class;
    }

    private boolean isLongArray2() {
        return value.getClass() == long[].class;
    }
    
    private boolean isBooleanArray() {
        return value.getClass() == Boolean[].class;
    }

    private boolean isBooleanArray2() {
        return value.getClass() == boolean[].class;
    }

    private boolean isStringArray() {
        return value.getClass() == String[].class;
    }
}