package frc.robot.util;

import java.util.Arrays;
import java.util.function.Supplier;

import edu.wpi.first.util.struct.StructSerializable;

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

            if(value == null) {
                // Do nothing; we don't need to record null
            } else if (isInteger()) {
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
            } else if (isFloatArray()) {
                // For some reason, Java does not have FloatStreams or mapToFloat
                Float[] a = (Float[]) value;
                float[] array = new float[a.length];
                for(int i = 0; i < a.length; i++){
                    array[i] = a[i];
                }
                LogManager.log(name, array);
            } else if (isFloatArray2()) {
                LogManager.log(name, (float[]) value);
            } else if (isBooleanArray()) {
                // For some reason, Java does not have BooleanStreams or mapToBoolean
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
            } else if (isEnum()) {
                LogManager.log(name, (Enum<?>) value);
            } else if (isEnumArray()) {
                LogManager.log(name, (Enum[]) value);
            } else if (isStruct()) {
                LogManager.log(name, (StructSerializable) value);
            } else if (isStructArray()) {
                LogManager.log(name, (StructSerializable[]) value);
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
        return value instanceof Integer;
    }

    private boolean isDouble() {
        return value instanceof Double;
    }

    private boolean isLong() {
        return value instanceof Long;
    }

    private boolean isBoolean() {
        return value instanceof Boolean;
    }

    private boolean isString() {
        return value instanceof String;
    }

    private boolean isIntegerArray() {
        return value instanceof Integer[];
    }

    private boolean isIntArray() {
        return value instanceof int[];
    }

    private boolean isDoubleArray() {
        return value instanceof Double[];
    }

    private boolean isDoubleArray2() {
        return value instanceof double[];
    }

    private boolean isLongArray() {
        return value instanceof Long[];
    }

    private boolean isLongArray2() {
        return value instanceof long[];
    }
    
    private boolean isBooleanArray() {
        return value instanceof Boolean[];
    }

    private boolean isBooleanArray2() {
        return value instanceof boolean[];
    }

    private boolean isStringArray() {
        return value instanceof String[];
    }

    private boolean isFloatArray() {
        return value instanceof Float[];
    }

    private boolean isFloatArray2() {
        return value instanceof float[];
    }

    private boolean isEnum() {
        return value instanceof Enum;
    }

    private boolean isEnumArray() {
        return value instanceof Enum[];
    }

    private boolean isStruct() {
        return value instanceof StructSerializable;
    }

    private boolean isStructArray() {
        return value instanceof StructSerializable[];
    }
}