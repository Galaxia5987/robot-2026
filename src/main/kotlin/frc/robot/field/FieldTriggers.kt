package frc.robot.field

import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.drive
import frc.robot.lib.extensions.not

var inAllianceZone = Trigger {
    ALLIANCE_ZONE_RECTANGLE.contains(drive.pose.translation)
}

var outOfAllianceZoneBelowCrossLine: Trigger =
    Trigger { CROSS_LINE_RECTANGLE.contains(drive.pose.translation) }
        .and(!inAllianceZone)

var outOfAllianceZoneAboveCrossLine: Trigger =
    Trigger { !CROSS_LINE_RECTANGLE.contains(drive.pose.translation) }
        .and(!inAllianceZone)
