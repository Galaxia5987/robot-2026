package frc.robot.field_constants

import edu.wpi.first.math.geometry.Rectangle2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.units.measure.Distance
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

val ALLIANCE_ZONE_WIDTH: Distance = 4.03.m
val ALLIANCE_ZONE_HEIGHT: Distance = 8.07.m

val FIELD_WIDTH: Distance = 16.54.m
val FIELD_HEIGHT: Distance = 4.03.m

val ALLIANCE_ZONE
    get() =
        Rectangle2d(
            Translation2d(
                if (IS_RED) (FIELD_WIDTH - ALLIANCE_ZONE_WIDTH)
                else ALLIANCE_ZONE_WIDTH,
                ALLIANCE_ZONE_HEIGHT
            ),
            Translation2d(if (IS_RED) FIELD_WIDTH else 0.m, 0.m)
        )

val allianceColorTrigger = Trigger { IS_RED }.onTrue(flipAllianceLocation())

private fun flipAllianceLocation() =
    runOnce({
        HUB_LOCATION = HUB_LOCATION.flip()
        CLIMB_LOCATION = CLIMB_LOCATION.flip()
        OUTPOST_LOCATION = OUTPOST_LOCATION.flip()
        DEPOT_LOCATION = DEPOT_LOCATION.flip()
    })
