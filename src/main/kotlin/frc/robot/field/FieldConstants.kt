package frc.robot.field

import com.pathplanner.lib.util.FlippingUtil
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rectangle2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.wpilibj2.command.Commands.runOnce
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.TunablePose3d
import frc.robot.drive
import frc.robot.lib.IS_RED
import frc.robot.lib.extensions.deg
import frc.robot.lib.extensions.flip
import frc.robot.lib.extensions.m
import frc.robot.lib.extensions.mm
import org.team5987.annotation.LogLevel
import org.team5987.annotation.LoggedOutput

val ALLIANCE_ZONE_WIDTH: Distance = 4.03.m

val ALLIANCE_ZONE_HEIGHT: Distance = 8.07.m

var HUB_LOCATION = Translation2d(4620.41.mm, 4034.63.mm)
    private set
var CLIMB_LOCATION = Pose2d(4090.6.mm, 5457.8.mm, Rotation2d(90.deg))
    private set

private val OFFSET_OF_FEED_LOCATION = Translation2d(700.mm, 700.mm)

var OUTPOST_LOCATION = OFFSET_OF_FEED_LOCATION
    private set
var DEPOT_LOCATION: Translation2d = Translation2d(OFFSET_OF_FEED_LOCATION.measureX, FlippingUtil.fieldSizeY.m - OFFSET_OF_FEED_LOCATION.measureY)
    private set

// For Debugging and tuning if necessary
//var temp = TunablePose3d(key="/Tuning/TempPose")
//    private set
//
//@LoggedOutput(LogLevel.COMP)
//fun getTempPose(): Pose3d = temp.get()



var ALLIANCE_ZONE =
    Rectangle2d(
        Translation2d(0.m, 0.m),
        Translation2d(ALLIANCE_ZONE_WIDTH, ALLIANCE_ZONE_HEIGHT)
    )
    private set

var OUTPOST_CROSS_LINE_RECTANGLE =
    Rectangle2d(
        Translation2d(),
        Translation2d(FlippingUtil.fieldSizeX, FlippingUtil.fieldSizeY / 2)
    )
    private set

val isCloserToDepotSide =
    Trigger { OUTPOST_CROSS_LINE_RECTANGLE.contains(drive.pose.translation) }
        .negate()

val allianceColorTrigger = Trigger { IS_RED }.onChange(flipAllianceLocation())

private fun flipAllianceLocation() =
    runOnce({
        HUB_LOCATION = HUB_LOCATION.flip()
        CLIMB_LOCATION = CLIMB_LOCATION.flip()
        OUTPOST_LOCATION = OUTPOST_LOCATION.flip()
        DEPOT_LOCATION = DEPOT_LOCATION.flip()
        ALLIANCE_ZONE = ALLIANCE_ZONE.flip()
        OUTPOST_CROSS_LINE_RECTANGLE = OUTPOST_CROSS_LINE_RECTANGLE.flip()
    })
