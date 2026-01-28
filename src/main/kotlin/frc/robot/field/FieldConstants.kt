package frc.robot.field

import com.pathplanner.lib.util.FlippingUtil
import edu.wpi.first.math.geometry.Rectangle2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj2.command.Commands.runOnce
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.lib.IS_RED
import frc.robot.lib.extensions.flip
import frc.robot.lib.extensions.m
import frc.robot.lib.extensions.mm

var HUB_LOCATION = Translation2d(4572.mm, 7632.7.mm)
var CLIMB_LOCATION = Translation2d(4090.6.mm, 5457.8.mm)
var OUTPOST_LOCATION = Translation2d(3026.5.mm, 7940.6.mm)
var DEPOT_LOCATION = Translation2d(3692.8.mm, 2641.6.mm)

var ALLIANCE_ZONE_RECTANGLE =
    Rectangle2d(
        Translation2d(),
        Translation2d(3972.7.mm, FlippingUtil.fieldSizeY.m)
    )
var CROSS_LINE_RECTANGLE =
    Rectangle2d(
        Translation2d(),
        Translation2d(FlippingUtil.fieldSizeX, FlippingUtil.fieldSizeY / 2)
    )

val allianceColorTrigger = Trigger { IS_RED }.onTrue(flipAllianceLocation())

private fun flipAllianceLocation() =
    runOnce({
        HUB_LOCATION = HUB_LOCATION.flip()
        CLIMB_LOCATION = CLIMB_LOCATION.flip()
        OUTPOST_LOCATION = OUTPOST_LOCATION.flip()
        DEPOT_LOCATION = DEPOT_LOCATION.flip()

        ALLIANCE_ZONE_RECTANGLE = ALLIANCE_ZONE_RECTANGLE.flip()
        CROSS_LINE_RECTANGLE = CROSS_LINE_RECTANGLE.flip()
    })
