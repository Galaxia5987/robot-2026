package frc.robot.field

import com.pathplanner.lib.util.FlippingUtil
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rectangle2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.wpilibj2.command.Commands.runOnce
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.drive
import frc.robot.lib.IS_RED
import frc.robot.lib.extensions.deg
import frc.robot.lib.extensions.flip
import frc.robot.lib.extensions.m
import frc.robot.lib.extensions.mm

var HUB_LOCATION = Translation2d(4572.mm, 7632.7.mm)
var CLIMB_LOCATION = Pose2d(4090.6.mm, 5457.8.mm, Rotation2d(90.deg))
var OUTPOST_LOCATION = Translation2d(3026.5.mm, 7940.6.mm)
var DEPOT_LOCATION = Translation2d(3692.8.mm, 2641.6.mm)

val ALLIANCE_ZONE_WIDTH: Distance = 4.03.m
val ALLIANCE_ZONE_HEIGHT: Distance = 8.07.m

val isCloserToDepot = Trigger {
    if (IS_RED) drive.pose.translation.y <= FlippingUtil.fieldSizeY / 2
    else drive.pose.translation.y > FlippingUtil.fieldSizeY / 2
} // TODO: Change

val FIELD_WIDTH: Distance = FlippingUtil.fieldSizeX.m
val FIELD_HEIGHT: Distance = FlippingUtil.fieldSizeY.m

private object AllianceZone {
    val BLUE_ALLIANCE_ZONE =
        Rectangle2d(
            Translation2d(0.m, 0.m),
            Translation2d(ALLIANCE_ZONE_WIDTH, ALLIANCE_ZONE_HEIGHT)
        )

    val RED_ALLIANCE_ZONE =
        Rectangle2d(
            Translation2d(FIELD_WIDTH - ALLIANCE_ZONE_WIDTH, 0.m),
            Translation2d(FIELD_WIDTH, ALLIANCE_ZONE_HEIGHT)
        )
}

val ALLIANCE_ZONE
    get() =
        if (IS_RED) AllianceZone.RED_ALLIANCE_ZONE
        else AllianceZone.BLUE_ALLIANCE_ZONE // TODO: Change

val allianceColorTrigger = Trigger { IS_RED }.onTrue(flipAllianceLocation())

private fun flipAllianceLocation() =
    runOnce({
        HUB_LOCATION = HUB_LOCATION.flip()
        CLIMB_LOCATION = CLIMB_LOCATION.flip()
        OUTPOST_LOCATION = OUTPOST_LOCATION.flip()
        DEPOT_LOCATION = DEPOT_LOCATION.flip()
    })
