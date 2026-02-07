package frc.robot.field

import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.drive
import frc.robot.isOurHubActive

val inAllianceZone = Trigger { ALLIANCE_ZONE.contains(drive.pose.translation) }

val isHubActive = Trigger { isOurHubActive }
