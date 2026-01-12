package frc.robot.lib.math.differential;

/*
This class contains a boolean trigger.
This trigger is used to detect when a boolean value changes from false to true or from true to false.
 */
public class BooleanTrigger {

    // The value to check
    private boolean value = false;

    // Whether the trigger has been triggered
    private boolean triggered = false;
    // Whether the trigger has been released
    private boolean released = false;

    /** Constructor for BooleanTrigger. */
    public BooleanTrigger() {}

    /**
     * Updates the trigger.
     *
     * @param newValue The new value to check.
     */
    public void update(boolean newValue) {
        boolean lastValue = value;
        value = newValue;

        triggered = value && !lastValue;
        released = !value && lastValue;
    }

    /**
     * Gets whether the trigger has been triggered.
     *
     * @return Whether the trigger has been triggered.
     */
    public boolean triggered() {
        return triggered;
    }

    /**
     * Gets whether the trigger has been released.
     *
     * @return Whether the trigger has been released.
     */
    public boolean released() {
        return released;
    }
}
