package frc.robot.field

import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.drive
import frc.robot.lib.IS_RED

var inAllianceZone = Trigger {(drive.pose.x < ALLIANCE_ZONE_LIMIT.x && !IS_RED) || (drive.pose.x > ALLIANCE_ZONE_LIMIT.x && IS_RED)}
var outOfAllianceZone_BelowCrossLine = Trigger {(drive.pose.x > ALLIANCE_ZONE_LIMIT.x && !IS_RED && drive.pose.y < 4.6) || (drive.pose.x < ALLIANCE_ZONE_LIMIT.x && IS_RED && drive.pose.y < 4.6)}
var outOfAllianceZone_AboveCrossLine = Trigger {(drive.pose.x > ALLIANCE_ZONE_LIMIT.x && !IS_RED && drive.pose.y > 4.6) || (drive.pose.x < ALLIANCE_ZONE_LIMIT.x && IS_RED && drive.pose.y > 4.6)}
