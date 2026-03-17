package frc.robot.field

import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.drive
import frc.robot.isOurHubActive
import frc.robot.lib.extensions.logTrigger

const val FIELD_LOGGING_PATH = "Field"

val inAllianceZone =
    Trigger { ALLIANCE_ZONE.contains(drive.pose.translation) }.and { !CLIMB_RECTANGLE.contains(drive.pose.translation)}
        .logTrigger("$FIELD_LOGGING_PATH/inAllianceZone")

val isCloserToDepotSide: Trigger =
    Trigger { OUTPOST_CROSS_LINE_RECTANGLE.contains(drive.pose.translation) }
        .negate()
        .logTrigger("$FIELD_LOGGING_PATH/isCloserToDepotSide")

val isHubActive =
    Trigger { isOurHubActive }.logTrigger("$FIELD_LOGGING_PATH/isHubActive")
