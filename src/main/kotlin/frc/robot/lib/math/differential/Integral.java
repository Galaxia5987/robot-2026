package frc.robot.lib.math.differential;

import edu.wpi.first.wpilibj.Timer;

/*
This class contains an integral.
This integral is used to approximate the area under a curve over time.
 */
public class Integral {

    // The value to integrate
    private double value;

    // The current value of the integral
    private double sum;

    // The last time the value was updated
    private double lastTimestamp = 0;

    // The integral of the integral
    private Integral integral = null;

    /** Constructor for Integral. */
    public Integral() {}

    /**
     * Gets the integral of the integral.
     *
     * @return The integral of the integral.
     */
    public Integral integrate() {
        if (integral == null) {
            integral = new Integral();
        }
        return integral;
    }

    /**
     * Updates the value to integrate.
     *
     * @param newValue The new value to integrate.
     */
    public void update(double newValue) {
        update(newValue, Timer.getFPGATimestamp());
    }

    /**
     * Updates the value to integrate.
     *
     * @param newValue The new value to integrate.
     * @param timestamp The current timestamp. [s]
     */
    public void update(double newValue, double timestamp) {
        sum += (value + newValue) * (timestamp - lastTimestamp) / 2;
        value = newValue;

        if (integral != null) {
            integral.update(sum, timestamp);
        }

        lastTimestamp = timestamp;
    }

    /**
     * Overrides the value of the integral.
     *
     * @param value The new value of the integral.
     */
    public void override(double value) {
        this.sum = value;
    }

    /**
     * Gets the value of the integral.
     *
     * @return The value of the integral.
     */
    public double get() {
        return sum;
    }
}
