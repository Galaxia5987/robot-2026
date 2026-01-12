package frc.robot.lib;

import static frc.robot.ConstantsKt.LOOP_TIME;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import java.util.Comparator;
import java.util.List;

public class Utils {
    public static final double EPSILON = 1e-9;

    public static boolean epsilonEquals(double a, double b) {
        return epsilonEquals(a, b, EPSILON);
    }

    public static boolean epsilonEquals(double a, double b, double maxError) {
        return Math.abs(a - b) <= maxError;
    }

    public static double normalize(double angleRadians) {
        while (angleRadians < 0) {
            angleRadians += 2 * Math.PI;
        }
        return angleRadians % (2 * Math.PI);
    }

    public static Rotation2d normalize(Rotation2d angle) {
        return Rotation2d.fromRadians(normalize(angle.getRadians()));
    }

    public static Angle normalize(Angle angle) {
        return Units.Radians.of(normalize(angle.in(Units.Radians)));
    }

    public static double getDistanceFromPoint(Translation2d point, Pose2d robotPose) {
        return robotPose.getTranslation().getDistance(point);
    }

    public static Pose2d calcClosestPose(List<Pose2d> points, Pose2d robotPose) {
        return points.stream()
                .min(
                        Comparator.comparingDouble(
                                point -> getDistanceFromPoint(point.getTranslation(), robotPose)))
                .orElse(null);
    }

    public static Rotation2d calcRotationToTranslation(
            Translation2d currentTranslation, Translation2d destinationTranslation) {
        return new Rotation2d(
                destinationTranslation.getX() - currentTranslation.getX(),
                destinationTranslation.getY() - currentTranslation.getY());
    }

    /** Logical inverse of the above. */
    public static ChassisSpeeds log(final ChassisSpeeds speeds) {
        var transform =
                new Pose2d(
                        speeds.vxMetersPerSecond * LOOP_TIME,
                        speeds.vyMetersPerSecond * LOOP_TIME,
                        new Rotation2d(speeds.omegaRadiansPerSecond * LOOP_TIME));
        final double dtheta = transform.getRotation().getRadians();
        final double half_dtheta = 0.5 * dtheta;
        final double cos_minus_one = transform.getRotation().getCos() - 1.0;
        double halftheta_by_tan_of_halfdtheta;
        if (Math.abs(cos_minus_one) < EPSILON) {
            halftheta_by_tan_of_halfdtheta = 1.0 - 1.0 / 12.0 * dtheta * dtheta;
        } else {
            halftheta_by_tan_of_halfdtheta =
                    -(half_dtheta * transform.getRotation().getSin()) / cos_minus_one;
        }
        final Translation2d translation_part =
                transform
                        .getTranslation()
                        .rotateBy(new Rotation2d(halftheta_by_tan_of_halfdtheta, -half_dtheta));
        return new ChassisSpeeds(
                translation_part.getX() / LOOP_TIME,
                translation_part.getY() / LOOP_TIME,
                dtheta / LOOP_TIME);
    }

    public static double distanceToSpeakerVarianceFactor(Translation2d toSpeaker) {
        return Math.cos(0.2 * toSpeaker.getY() / toSpeaker.getX());
    }
}
