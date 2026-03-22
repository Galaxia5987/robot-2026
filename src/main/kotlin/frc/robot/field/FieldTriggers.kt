package frc.robot.field

import edu.wpi.first.units.measure.Angle
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.drive
import frc.robot.isOurHubPreActive
import frc.robot.lib.IS_RED
import frc.robot.lib.extensions.deg
import frc.robot.lib.extensions.logTrigger
import frc.robot.lib.extensions.mirror
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

val isInDoubleFeedingZone: Trigger = Trigger { DOUBLE_FEEDING_RECTANGLE.contains(drive.pose.translation) }.or {
    DOUBLE_FEEDING_RECTANGLE.mirror().contains(drive.pose.translation)
}.logTrigger("$FIELD_LOGGING_PATH/isInDoubleFeedingZone")

val rotationForDoubleFeeding: Angle
    get() = if (IS_RED) 0.deg else 180.deg

val ROTATION_TOLERANCE = 30.deg

val isInDoubleFeedingRotation: Trigger = Trigger ({drive.pose.rotation.measure.isNear(rotationForDoubleFeeding, ROTATION_TOLERANCE)}).logTrigger("$FIELD_LOGGING_PATH/isInDoubleFeedingRotation")

val isHubActive =
    Trigger { isOurHubPreActive }.logTrigger("$FIELD_LOGGING_PATH/isHubActive")
