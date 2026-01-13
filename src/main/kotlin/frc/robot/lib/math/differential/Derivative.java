package frc.robot.lib.math.differential;

import edu.wpi.first.wpilibj.Timer;

/*
This class represents the derivative of a value over time.
 */
public class Derivative {

    // The value to differentiate
    private double value = 0;

    // The derivative of the value
    private double slope;

    // The last time the value was updated
    private double lastTimestamp = 0;

    // The derivative of the derivative
    private Derivative derivative = null;

    /** Constructor for Derivative. */
    public Derivative() {}

    /**
     * Gets the derivative of the derivative.
     *
     * @return The derivative of the derivative.
     */
    public Derivative differentiate() {
        if (derivative == null) {
            derivative = new Derivative();
        }
        return derivative;
    }

    /**
     * Updates the value to differentiate.
     *
     * @param newValue The new value to differentiate.
     */
    public void update(double newValue) {
        update(newValue, Timer.getFPGATimestamp());
    }

    /**
     * Updates the value to differentiate.
     *
     * @param newValue The new value to differentiate.
     * @param timestamp The current timestamp. [s]
     */
    public void update(double newValue, double timestamp) {
        double lastValue = value;
        value = newValue;

        slope = (value - lastValue) / (timestamp - lastTimestamp);

        if (derivative != null) {
            derivative.update(slope, timestamp);
        }

        lastTimestamp = timestamp;
    }

    /**
     * Gets the derivative of the value.
     *
     * @return The derivative of the value.
     */
    public double get() {
        return slope;
    }
}
