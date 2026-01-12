package frc.robot.lib.math.interpolation;

public class InterpolatingDoubleMap
        extends InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> {

    public InterpolatingDoubleMap(int maximumSize) {
        super(maximumSize);
    }

    public InterpolatingDoubleMap() {}

    public InterpolatingDouble put(double a, double b) {
        return super.put(new InterpolatingDouble(a), new InterpolatingDouble(b));
    }
}
