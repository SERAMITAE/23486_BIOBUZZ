package util;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.HashMap;

@Configurable
public class CachedMotor implements DcMotorEx {
    public static double filterCoefficient = 1;
    public static double filterFactor = 4;
    public static double filterScaleFactor = -0.000005;

    private static final double POWER_EPSILON = 0.01;
    private static final double HOLD_POWER = 0.2;
    private static final HashMap<String, DcMotorEx> created = new HashMap<>();

    public enum MotorState {
        IDLE,
        MOVING,
        HOLDING,
        STALLED
    }

    public static final class Configuration {
        private double stallCurrentAmps = Double.POSITIVE_INFINITY;
        private double stallVelocity = 5;
        private double stallPower = 0.2;
        private int tolerance = 10;
        private int historySize = 64;
        private double currentFilterCoefficient = filterCoefficient;
        private double currentFilterFactor = filterFactor;
        private double currentFilterScaleFactor = filterScaleFactor;

        private Configuration() {
        }

        public double getStallCurrent() {
            return stallCurrentAmps;
        }

        public double getStallVelocity() {
            return stallVelocity;
        }

        public double getStallPower() {
            return stallPower;
        }

        public int getTolerance() {
            return tolerance;
        }

        public int getHistorySize() {
            return historySize;
        }

        public double getCurrentFilterCoefficient() {
            return currentFilterCoefficient;
        }

        public double getCurrentFilterFactor() {
            return currentFilterFactor;
        }

        public double getCurrentFilterScaleFactor() {
            return currentFilterScaleFactor;
        }
    }

    public final class Statistics {
        private double peakCurrent;
        private double currentTotal;
        private double peakVelocity;
        private double velocityTotal;
        private double peakPower;
        private double powerTotal;
        private long sampleCount;

        private Statistics() {
        }

        private void addSample(double current, double velocity, double appliedPower) {
            peakCurrent = Math.max(peakCurrent, current);
            currentTotal += current;
            peakVelocity = Math.max(peakVelocity, velocity);
            velocityTotal += velocity;
            peakPower = Math.max(peakPower, appliedPower);
            powerTotal += appliedPower;
            sampleCount++;
        }

        private void reset() {
            peakCurrent = 0;
            currentTotal = 0;
            peakVelocity = 0;
            velocityTotal = 0;
            peakPower = 0;
            powerTotal = 0;
            sampleCount = 0;
        }

        public double getPeakCurrent() {
            return peakCurrent;
        }

        public double getAverageCurrent() {
            return sampleCount == 0 ? 0 : currentTotal / sampleCount;
        }

        public double getPeakVelocity() {
            return peakVelocity;
        }

        public double getAverageVelocity() {
            return sampleCount == 0 ? 0 : velocityTotal / sampleCount;
        }

        public double getPeakPower() {
            return peakPower;
        }

        public double getAveragePower() {
            return sampleCount == 0 ? 0 : powerTotal / sampleCount;
        }

        public double getRuntimeSeconds() {
            return CachedMotor.this.getRuntimeSeconds();
        }

        public long getUpdateCount() {
            return CachedMotor.this.getUpdateCount();
        }
    }

    public final DcMotorEx motor;

    private double targetVelocity;
    private int targetPosition;
    private double power;
    private double appliedPower;

    private RunMode mode;
    private ZeroPowerBehavior zeroPowerBehavior;
    private PIDFCoefficients velocityPIDFCoefficients;
    private PIDFCoefficients positionPIDFCoefficients;
    private Direction direction;

    private int currentCacheTimeMS = 200;
    private long lastCurrentSampleNanos;
    private double rawCurrentMilliamps;
    private double filteredCurrentMilliamps;
    private boolean hasCurrentSample;

    private final Configuration configuration = new Configuration();
    private final Statistics statistics = new Statistics();
    private final ScalingLowPassFilter currentFilter = new ScalingLowPassFilter(
            configuration.currentFilterCoefficient,
            configuration.currentFilterFactor,
            configuration.currentFilterScaleFactor
    );

    private double[] currentHistory = new double[0];
    private double[] velocityHistory = new double[0];
    private double[] powerHistory = new double[0];
    private int[] positionHistory = new int[0];
    private int historyIndex;
    private int historyCount;

    private MotorState motorState = MotorState.IDLE;
    private boolean holding;
    private RunMode heldMode;
    private double heldPower;
    private boolean stalled;
    private boolean wasStalled;
    private boolean wasAtTarget;
    private boolean wasOverCurrent;
    private Runnable stallCallback;
    private Runnable targetReachedCallback;
    private Runnable overCurrentCallback;

    private int lastPosition;
    private long lastPositionSampleNanos;
    private boolean hasPositionSample;
    private double estimatedVelocity;
    private long runtimeStartNanos;
    private long updateCount;

    public CachedMotor(DcMotorEx motor) {
        this.motor = motor;
        _init();
    }

    public CachedMotor(HardwareMap hardwareMap, String name) {
        this.motor = hardwareMap.get(DcMotorEx.class, name);
        CachedMotor.created.put(name, motor);
        _init();
    }

    private void _init() {
        targetVelocity = 0;
        targetPosition = motor.getTargetPosition();
        power = motor.getPower();
        appliedPower = power;
        mode = motor.getMode();
        zeroPowerBehavior = motor.getZeroPowerBehavior();
        velocityPIDFCoefficients = motor.getPIDFCoefficients(RunMode.RUN_USING_ENCODER);
        positionPIDFCoefficients = motor.getPIDFCoefficients(RunMode.RUN_TO_POSITION);
        direction = motor.getDirection();
        configuration.tolerance = motor.getTargetPositionTolerance();
        lastPosition = motor.getCurrentPosition();
        hasPositionSample = true;
        wasAtTarget = isWithinTolerance(lastPosition);
        long now = System.nanoTime();
        lastPositionSampleNanos = now;
        runtimeStartNanos = now;
        resizeHistory(configuration.historySize);
    }

    public void update() {
        long now = System.nanoTime();
        int currentPosition = motor.getCurrentPosition();
        if (hasPositionSample) {
            long elapsedNanos = now - lastPositionSampleNanos;
            if (elapsedNanos > 0) {
                estimatedVelocity = (currentPosition - lastPosition) * 1_000_000_000.0 / elapsedNanos;
            }
        } else {
            hasPositionSample = true;
            estimatedVelocity = 0;
        }
        lastPosition = currentPosition;
        lastPositionSampleNanos = now;

        refreshCurrent(now);
        boolean atTarget = isWithinTolerance(currentPosition);
        boolean overCurrent = motor.isOverCurrent();
        stalled = Math.abs(power) >= configuration.stallPower
                && Math.abs(estimatedVelocity) <= configuration.stallVelocity
                && filteredCurrentMilliamps / 1000.0 >= configuration.stallCurrentAmps;

        if (stalled) {
            motorState = MotorState.STALLED;
        } else if (holding) {
            motorState = MotorState.HOLDING;
        } else if (Math.abs(power) >= POWER_EPSILON || motor.isBusy() || Math.abs(estimatedVelocity) > configuration.stallVelocity) {
            motorState = MotorState.MOVING;
        } else {
            motorState = MotorState.IDLE;
        }

        addHistory(filteredCurrentMilliamps, estimatedVelocity, power, currentPosition);
        statistics.addSample(
                Math.abs(filteredCurrentMilliamps) / 1000.0,
                Math.abs(estimatedVelocity),
                Math.abs(power)
        );
        updateCount++;

        if (stalled && !wasStalled && stallCallback != null) {
            stallCallback.run();
        }
        if (atTarget && !wasAtTarget && targetReachedCallback != null) {
            targetReachedCallback.run();
        }
        if (overCurrent && !wasOverCurrent && overCurrentCallback != null) {
            overCurrentCallback.run();
        }
        wasStalled = stalled;
        wasAtTarget = atTarget;
        wasOverCurrent = overCurrent;
    }

    private void applyPower(double value) {
        if (Math.abs(appliedPower - value) < POWER_EPSILON) {
            return;
        }
        appliedPower = value;
        motor.setPower(value);
    }

    private void refreshCurrent(long now) {
        long cacheNanos = currentCacheTimeMS * 1_000_000L;
        if (hasCurrentSample && cacheNanos > 0 && now - lastCurrentSampleNanos < cacheNanos) {
            return;
        }
        rawCurrentMilliamps = motor.getCurrent(CurrentUnit.MILLIAMPS);
        filteredCurrentMilliamps = currentFilter.update(rawCurrentMilliamps);
        lastCurrentSampleNanos = now;
        hasCurrentSample = true;
    }

    private void resizeHistory(int size) {
        currentHistory = new double[size];
        velocityHistory = new double[size];
        powerHistory = new double[size];
        positionHistory = new int[size];
        historyIndex = 0;
        historyCount = 0;
    }

    private void addHistory(double current, double velocity, double appliedPower, int position) {
        if (configuration.historySize == 0) {
            return;
        }
        currentHistory[historyIndex] = current;
        velocityHistory[historyIndex] = velocity;
        powerHistory[historyIndex] = appliedPower;
        positionHistory[historyIndex] = position;
        historyIndex = (historyIndex + 1) % configuration.historySize;
        if (historyCount < configuration.historySize) {
            historyCount++;
        }
    }

    private boolean isWithinTolerance(int currentPosition) {
        return Math.abs((long) targetPosition - currentPosition) <= configuration.tolerance;
    }

    public Configuration getConfiguration() {
        return configuration;
    }

    public Statistics getStatistics() {
        return statistics;
    }

    public MotorState getMotorState() {
        return motorState;
    }

    public void setStallCurrent(double currentAmps) {
        if (currentAmps < 0) {
            throw new IllegalArgumentException("stall current must be non-negative");
        }
        configuration.stallCurrentAmps = currentAmps;
    }

    public void setStallCurrent(double current, CurrentUnit unit) {
        setStallCurrent(unit == CurrentUnit.MILLIAMPS ? current / 1000.0 : current);
    }

    public void setStallVelocity(double velocity) {
        if (velocity < 0) {
            throw new IllegalArgumentException("stall velocity must be non-negative");
        }
        configuration.stallVelocity = velocity;
    }

    public void setStallPower(double power) {
        if (power < 0 || power > 1) {
            throw new IllegalArgumentException("stall power must be between 0 and 1");
        }
        configuration.stallPower = power;
    }

    public void setTolerance(int tolerance) {
        setTargetPositionTolerance(tolerance);
    }

    public void setHistorySize(int historySize) {
        if (historySize < 0) {
            throw new IllegalArgumentException("history size must be non-negative");
        }
        configuration.historySize = historySize;
        resizeHistory(historySize);
    }

    public int getHistorySize() {
        return configuration.historySize;
    }

    public int getHistoryCount() {
        return historyCount;
    }

    public void holdPosition() {
        if (holding) {
            return;
        }
        heldMode = mode;
        heldPower = power;
        setTargetPosition(motor.getCurrentPosition());
        setMode(RunMode.RUN_TO_POSITION);
        holding = true;
        setPower(Math.abs(heldPower) >= POWER_EPSILON ? Math.abs(heldPower) : HOLD_POWER);
    }

    public void releaseHold() {
        if (!holding) {
            return;
        }
        holding = false;
        setMode(heldMode);
        setPower(heldPower);
    }

    public boolean isHolding() {
        return holding;
    }

    public int getError() {
        long error = (long) targetPosition - motor.getCurrentPosition();
        return error > Integer.MAX_VALUE ? Integer.MAX_VALUE : error < Integer.MIN_VALUE ? Integer.MIN_VALUE : (int) error;
    }

    public boolean atTarget() {
        return isWithinTolerance(motor.getCurrentPosition());
    }

    public boolean isStalled() {
        return stalled;
    }

    public boolean isMoving() {
        return motorState == MotorState.MOVING;
    }

    public boolean isIdle() {
        return motorState == MotorState.IDLE;
    }

    public double getEstimatedVelocity() {
        return estimatedVelocity;
    }

    public double getRuntimeSeconds() {
        return (System.nanoTime() - runtimeStartNanos) / 1_000_000_000.0;
    }

    public long getUpdateCount() {
        return updateCount;
    }

    public void resetRuntime() {
        runtimeStartNanos = System.nanoTime();
    }

    public void resetStatistics() {
        statistics.reset();
        updateCount = 0;
        resetRuntime();
    }

    public double getPeakCurrent() {
        return statistics.getPeakCurrent();
    }

    public double getAverageCurrent() {
        return statistics.getAverageCurrent();
    }

    public double getPeakVelocity() {
        return statistics.getPeakVelocity();
    }

    public double getAverageVelocity() {
        return statistics.getAverageVelocity();
    }

    public double getPeakPower() {
        return statistics.getPeakPower();
    }

    public double getAveragePower() {
        return statistics.getAveragePower();
    }

    public void onStall(Runnable callback) {
        stallCallback = callback;
    }

    public void onTargetReached(Runnable callback) {
        targetReachedCallback = callback;
    }

    public void onOverCurrent(Runnable callback) {
        overCurrentCallback = callback;
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Motor State", motorState);
        telemetry.addData("Power", power);
        telemetry.addData("Velocity", getVelocity());
        telemetry.addData("Estimated Velocity", estimatedVelocity);
        telemetry.addData("Current", getRawCurrent(CurrentUnit.AMPS));
        telemetry.addData("Filtered Current", getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Encoder Position", getCurrentPosition());
        telemetry.addData("Target Position", targetPosition);
        telemetry.addData("Error", getError());
        telemetry.addData("Busy", isBusy());
        telemetry.addData("Holding", holding);
        telemetry.addData("Stalled", stalled);
        telemetry.addData("OverCurrent", isOverCurrent());
        telemetry.addData("Peak Current", getPeakCurrent());
        telemetry.addData("Average Current", getAverageCurrent());
        telemetry.addData("Peak Velocity", getPeakVelocity());
        telemetry.addData("Average Velocity", getAverageVelocity());
        telemetry.addData("Peak Power", getPeakPower());
        telemetry.addData("Average Power", getAveragePower());
        telemetry.addData("Runtime", getRuntimeSeconds());
        telemetry.addData("Update Count", updateCount);
    }

    @Override
    public void setMotorEnable() {
        motor.setMotorEnable();
    }

    @Override
    public void setMotorDisable() {
        motor.setMotorDisable();
    }

    @Override
    public boolean isMotorEnabled() {
        return motor.isMotorEnabled();
    }

    @Override
    public void setVelocity(double angularRate) {
        if (targetVelocity == angularRate) {
            return;
        }
        targetVelocity = angularRate;
        motor.setVelocity(angularRate);
    }

    @Override
    @Deprecated
    public void setVelocity(double angularRate, AngleUnit unit) {
        motor.setVelocity(angularRate, unit);
    }

    public double getTargetVelocity() {
        return targetVelocity;
    }

    @Override
    public double getVelocity() {
        return motor.getVelocity();
    }

    @Override
    @Deprecated
    public double getVelocity(AngleUnit unit) {
        return motor.getVelocity(unit);
    }

    @Override
    @Deprecated
    public void setPIDCoefficients(RunMode mode, PIDCoefficients pidCoefficients) {
        PIDFCoefficients coefficients = getPIDFCoefficients(mode);
        setPIDFCoefficients(mode, new PIDFCoefficients(pidCoefficients.p, pidCoefficients.i, pidCoefficients.d, coefficients.f));
    }

    @Override
    public void setPIDFCoefficients(RunMode mode, PIDFCoefficients pidfCoefficients) throws UnsupportedOperationException {
        switch (mode) {
            case RUN_TO_POSITION:
                positionPIDFCoefficients = pidfCoefficients;
                break;
            case RUN_USING_ENCODER:
                velocityPIDFCoefficients = pidfCoefficients;
                break;
            default:
                break;
        }
        motor.setPIDFCoefficients(mode, pidfCoefficients);
    }

    public void setVelocityPIDFCoefficients(PIDFCoefficients pidfCoefficients) {
        setPIDFCoefficients(RunMode.RUN_USING_ENCODER, pidfCoefficients);
    }

    @Override
    public void setVelocityPIDFCoefficients(double p, double i, double d, double f) {
        setPIDFCoefficients(RunMode.RUN_USING_ENCODER, new PIDFCoefficients(p, i, d, f));
    }

    @Override
    @Deprecated
    public void setPositionPIDFCoefficients(double p) {
        PIDFCoefficients coefficients = getPIDFCoefficients(RunMode.RUN_TO_POSITION);
        setPIDFCoefficients(RunMode.RUN_TO_POSITION, new PIDFCoefficients(p, coefficients.i, coefficients.d, coefficients.f));
    }

    public void setPositionPIDFCoefficients(PIDFCoefficients pidfCoefficients) {
        setPIDFCoefficients(RunMode.RUN_TO_POSITION, pidfCoefficients);
    }

    public void setPositionPIDFCoefficients(double p, double i, double d, double f) {
        setPIDFCoefficients(RunMode.RUN_TO_POSITION, new PIDFCoefficients(p, i, d, f));
    }

    @Override
    @Deprecated
    public PIDCoefficients getPIDCoefficients(RunMode mode) {
        PIDFCoefficients coefficients = getPIDFCoefficients(mode);
        return new PIDCoefficients(coefficients.p, coefficients.i, coefficients.d);
    }

    @Override
    public PIDFCoefficients getPIDFCoefficients(RunMode mode) {
        switch (mode) {
            case RUN_TO_POSITION:
                return positionPIDFCoefficients;
            case RUN_USING_ENCODER:
                return velocityPIDFCoefficients;
            default:
                return null;
        }
    }

    public PIDFCoefficients getVelocityPIDFCoefficients() {
        return velocityPIDFCoefficients;
    }

    public PIDFCoefficients getPositionPIDFCoefficients() {
        return positionPIDFCoefficients;
    }

    @Override
    public void setTargetPositionTolerance(int tolerance) {
        if (tolerance < 0) {
            throw new IllegalArgumentException("tolerance must be non-negative");
        }
        configuration.tolerance = tolerance;
        motor.setTargetPositionTolerance(tolerance);
    }

    @Override
    public int getTargetPositionTolerance() {
        return configuration.tolerance;
    }

    public void setCurrentCacheTime(int currentCacheTimeMS) {
        if (currentCacheTimeMS < 0) {
            throw new IllegalArgumentException("current cache time must be non-negative");
        }
        this.currentCacheTimeMS = currentCacheTimeMS;
    }

    public void setCurrentFilter(double coefficient, double factor, double scaleFactor) {
        configuration.currentFilterCoefficient = coefficient;
        configuration.currentFilterFactor = factor;
        configuration.currentFilterScaleFactor = scaleFactor;
        currentFilter.setValues(coefficient, factor, scaleFactor);
    }

    public void setCurrentFilter(TriTuple<Double, Double, Double> values) {
        setCurrentFilter(values.v1, values.v2, values.v3);
    }

    @Override
    public double getCurrent(CurrentUnit unit) {
        refreshCurrent(System.nanoTime());
        return unit == CurrentUnit.AMPS ? filteredCurrentMilliamps / 1000.0 : filteredCurrentMilliamps;
    }

    public double getRawCurrent(CurrentUnit unit) {
        refreshCurrent(System.nanoTime());
        return unit == CurrentUnit.AMPS ? rawCurrentMilliamps / 1000.0 : rawCurrentMilliamps;
    }

    @Override
    public double getCurrentAlert(CurrentUnit unit) {
        return motor.getCurrentAlert(unit);
    }

    @Override
    public void setCurrentAlert(double current, CurrentUnit unit) {
        motor.setCurrentAlert(current, unit);
    }

    @Override
    public boolean isOverCurrent() {
        return motor.isOverCurrent();
    }

    @Override
    public MotorConfigurationType getMotorType() {
        return motor.getMotorType();
    }

    @Override
    public void setMotorType(MotorConfigurationType motorType) {
        motor.setMotorType(motorType);
    }

    @Override
    public DcMotorController getController() {
        return motor.getController();
    }

    @Override
    public int getPortNumber() {
        return motor.getPortNumber();
    }

    @Override
    public void setZeroPowerBehavior(ZeroPowerBehavior zeroPowerBehavior) {
        setZeroPowerBehavior(zeroPowerBehavior, false);
    }

    public void setZeroPowerBehavior(ZeroPowerBehavior zeroPowerBehavior, boolean force) {
        if (!force && this.zeroPowerBehavior == zeroPowerBehavior) {
            return;
        }
        this.zeroPowerBehavior = zeroPowerBehavior;
        motor.setZeroPowerBehavior(zeroPowerBehavior);
    }

    @Override
    public ZeroPowerBehavior getZeroPowerBehavior() {
        return zeroPowerBehavior;
    }

    @Override
    @Deprecated
    public void setPowerFloat() {
        motor.setPowerFloat();
    }

    @Override
    public boolean getPowerFloat() {
        return motor.getPowerFloat();
    }

    @Override
    public void setTargetPosition(int position) {
        if (targetPosition == position) {
            return;
        }
        targetPosition = position;
        wasAtTarget = false;
        motor.setTargetPosition(position);
    }

    @Override
    public int getTargetPosition() {
        return targetPosition;
    }

    @Override
    public boolean isBusy() {
        return motor.isBusy();
    }

    @Override
    public int getCurrentPosition() {
        return motor.getCurrentPosition();
    }

    @Override
    public void setMode(RunMode mode) {
        this.mode = mode;
        motor.setMode(mode);
    }

    @Override
    public RunMode getMode() {
        return mode;
    }

    public void resetEncoder() {
        motor.setMode(RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(mode);
        targetPosition = motor.getTargetPosition();
        hasPositionSample = false;
        wasAtTarget = false;
    }

    @Override
    public void setDirection(Direction direction) {
        this.direction = direction;
        motor.setDirection(direction);
    }

    public void setInverted(boolean inverted) {
        setDirection(inverted ? Direction.REVERSE : Direction.FORWARD);
    }

    @Override
    public Direction getDirection() {
        return direction;
    }

    public boolean getInverted() {
        return getDirection() == Direction.REVERSE;
    }

    @Override
    public void setPower(double power) {
        this.power = MathFunctions.clamp(power, -1, 1);
        applyPower(this.power);
    }

    @Override
    public double getPower() {
        return power;
    }

    @Override
    public Manufacturer getManufacturer() {
        return motor.getManufacturer();
    }

    @Override
    public String getDeviceName() {
        return motor.getDeviceName();
    }

    @Override
    public String getConnectionInfo() {
        return motor.getConnectionInfo();
    }

    @Override
    public int getVersion() {
        return motor.getVersion();
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {
        motor.resetDeviceConfigurationForOpMode();
    }

    @Override
    public void close() {
        motor.close();
    }
}
