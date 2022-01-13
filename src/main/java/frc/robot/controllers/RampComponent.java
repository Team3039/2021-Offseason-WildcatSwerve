package frc.robot.controllers;

import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonProperty;
import edu.wpi.first.wpilibj.Timer;

import javax.annotation.Nonnull;
import javax.annotation.Nullable;
import java.util.function.DoubleUnaryOperator;

/**
 * A component for limiting the rate of change of a value.
 */
public class RampComponent implements DoubleUnaryOperator, Cloneable {

    /**
     * The maximum allowed change in the value per second.
     */
    private final double maxIncreasePerMillis, maxDecreasePerMillis;
    Timer timer;
    /**
     * The value most recently returned.
     */
    private double lastValue;
    /**
     * The time, in milliseconds, that the value most recently returned was returned at.
     */
    private double lastTime;

    /**
     * Default constructor.
     *
     * @param maxIncreasePerSecond The maximum allowed increase in the value per second.
     * @param maxDecreasePerSecond The maximum allowed decrease in the value per second. Should be
     *                             positive. Defaults to maxIncreasePerSecond.
     */
    @JsonCreator
    public RampComponent(
            @JsonProperty(required = true) double maxIncreasePerSecond,
            @Nullable Double maxDecreasePerSecond) {
        this.maxIncreasePerMillis = maxIncreasePerSecond / 1000.;
        this.maxDecreasePerMillis =
                maxDecreasePerSecond != null ? maxDecreasePerSecond / 1000. : maxIncreasePerMillis;
        timer = new Timer();
        timer.stop();
        timer.reset();
        timer.start();
    }

    /**
     * Ramp the given value.
     *
     * @param value The current value of whatever it is you're ramping
     * @return The ramped version of that value.
     */
    @Override
    public double applyAsDouble(double value) {
        if (value > lastValue) {
            lastValue =
                    Math.min(
                            value, lastValue + (timer.get() * 1000 - lastTime) * maxIncreasePerMillis);
        } else {
            lastValue =
                    Math.max(
                            value, lastValue - (timer.get() * 1000 - lastTime) * maxDecreasePerMillis);
        }
        lastTime = (timer.get() * 1000);
        return lastValue;
    }

    /**
     * Get an a copy of this object.
     *
     * @return a new {@link RampComponent} with the same max change per second
     */
    @Override
    @Nonnull
    public RampComponent clone() throws CloneNotSupportedException {
        return (RampComponent) super.clone();
    }
}