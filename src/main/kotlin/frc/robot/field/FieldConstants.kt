package frc.robot.field

import com.pathplanner.lib.util.FlippingUtil
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rectangle2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.units.measure.Distance
import frc.robot.lib.extensions.deg
import frc.robot.lib.extensions.flip
import frc.robot.lib.extensions.flipIfNeeded
import frc.robot.lib.extensions.m
import frc.robot.lib.extensions.mirror
import frc.robot.lib.extensions.mm
import frc.robot.subsystems.shooter.hood.CROUCH_TOLERANCE

val ALLIANCE_ZONE_WIDTH: Distance = 4.03.m

val ALLIANCE_ZONE_HEIGHT: Distance = 8.07.m

val HUB_TRANSLATION: Translation2d
    get() = Translation2d(4620.41.mm, 4034.63.mm).flipIfNeeded()

val CLIMB_TRANSLATION: Pose2d
    get() = Pose2d(4090.6.mm, 5457.8.mm, Rotation2d(90.deg)).flipIfNeeded()

private val OUTPOST_FEED_TRANSLATION = Translation2d(2.880.m, 1.407.m)

val OUTPOST_LOCATION: Translation2d
    get() = OUTPOST_FEED_TRANSLATION.flipIfNeeded()

val DEPOT_TRANSLATION: Translation2d
    get() =
        Translation2d(
                OUTPOST_FEED_TRANSLATION.measureX,
                FlippingUtil.fieldSizeY.m - OUTPOST_FEED_TRANSLATION.measureY
            )
            .flipIfNeeded()

val TRENCH_WIDTH = 1331.75.mm

private val TRENCH =
    Rectangle2d(
        Translation2d(HUB_TRANSLATION.x.m - CROUCH_TOLERANCE, 0.m),
        Translation2d(HUB_TRANSLATION.x.m + CROUCH_TOLERANCE, TRENCH_WIDTH)
    )

val TRENCH_AREAS: List<Rectangle2d>
    get() =
        listOf(TRENCH, TRENCH.flip(), TRENCH.mirror(), TRENCH.flip().mirror())
val ALLIANCE_ZONE: Rectangle2d
    get() =
        Rectangle2d(
                Translation2d(0.m, Int.MIN_VALUE.m),
                Translation2d(ALLIANCE_ZONE_WIDTH, Int.MAX_VALUE.m)
            )
            .flipIfNeeded()

val OUTPOST_CROSS_LINE_RECTANGLE: Rectangle2d
    get() =
        Rectangle2d(
                Translation2d(0.m, Int.MIN_VALUE.m),
                Translation2d(
                    FlippingUtil.fieldSizeX,
                    FlippingUtil.fieldSizeY / 2
                )
            )
            .flipIfNeeded()

// For Debugging and tuning if necessary
// var temp = TunablePose3d(key="/Tuning/TempPose")
//    private set
//
// @LoggedOutput(LogLevel.COMP)
// fun getTempPose(): Pose3d = temp.get()
