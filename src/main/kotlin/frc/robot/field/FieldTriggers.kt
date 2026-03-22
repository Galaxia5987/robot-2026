package frc.robot.field

import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.drive
import frc.robot.isOurHubPreActive
import frc.robot.lib.extensions.logTrigger
import frc.robot.subsystems.shooter.turret.turretTranslationFieldOriented

const val FIELD_LOGGING_PATH = "Field"

val inClimbRectangle = Trigger {
    CLIMB_RECTANGLE.contains(turretTranslationFieldOriented)
}

val inAllianceZone =
    Trigger { ALLIANCE_ZONE.contains(turretTranslationFieldOriented) }
        .and(inClimbRectangle.negate())
        .logTrigger("$FIELD_LOGGING_PATH/inAllianceZone")

val inExtendedAllianceZone =
    Trigger { EXTENDED_ALLIANCE_ZONE.contains(turretTranslationFieldOriented) }
        .and(inClimbRectangle.negate())
        .logTrigger("$FIELD_LOGGING_PATH/inExtendedAllianceZone")

val isCloserToDepotSide: Trigger =
    Trigger { OUTPOST_CROSS_LINE_RECTANGLE.contains(drive.pose.translation) }
        .negate()
        .logTrigger("$FIELD_LOGGING_PATH/isCloserToDepotSide")

val isHubActive =
    Trigger { isOurHubPreActive }.logTrigger("$FIELD_LOGGING_PATH/isHubActive")
