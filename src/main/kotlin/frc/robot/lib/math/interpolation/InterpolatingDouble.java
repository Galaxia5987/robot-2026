package frc.robot.lib.math.interpolation;

/**
 * A Double that can be interpolated using the InterpolatingTreeMap.
 *
 * @see InterpolatingTreeMap
 */
public class InterpolatingDouble
        implements Interpolable<InterpolatingDouble>,
                InverseInterpolable<InterpolatingDouble>,
                Comparable<InterpolatingDouble> {

    public double value;

    public InterpolatingDouble(double val) {
        value = val;
    }

    @Override
    public InterpolatingDouble interpolate(InterpolatingDouble other, double x) {
        double dy = other.value - this.value;
        return new InterpolatingDouble(dy * x + value);
    }

    @Override
    public double inverseInterpolate(InterpolatingDouble upper, InterpolatingDouble query) {
        double upperToLower = upper.value - value;
        if (upperToLower <= 0) {
            return 0;
        }
        double queryToLower = query.value - value;
        if (queryToLower <= 0) {
            return 0;
        }
        return queryToLower / upperToLower;
    }

    @Override
    public int compareTo(InterpolatingDouble other) {
        return Double.compare(value, other.value);
    }
}
