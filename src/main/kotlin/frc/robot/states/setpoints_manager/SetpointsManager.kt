package frc.robot.states.setpoints_manager

import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.units.Measure
import edu.wpi.first.units.Unit
import edu.wpi.first.wpilibj2.command.Commands.runOnce
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.field.DEPOT_LOCATION
import frc.robot.field.HUB_LOCATION
import frc.robot.field.OUTPOST_LOCATION
import frc.robot.field.inAllianceZone
import frc.robot.field.outOfAllianceZoneAboveCrossLine
import frc.robot.field.outOfAllianceZoneBelowCrossLine
import frc.robot.lib.extensions.not
import frc.robot.states.DriverOverrides
import frc.robot.states.setpoints_manager.setpoints_state.interpolationShootingMap
import frc.robot.states.setpoints_manager.setpoints_state.shootOnMoveMap
import frc.robot.states.setpoints_manager.setpoints_state.staticShootingMap
import org.team5987.annotation.LogLevel
import org.team5987.annotation.LoggedOutput

@LoggedOutput(LogLevel.COMP) var currentGoal: Translation2d = HUB_LOCATION

private val goalHubTrigger =
    inAllianceZone.onTrue(runOnce({ currentGoal = HUB_LOCATION }))

private val goalDepotTrigger =
    outOfAllianceZoneAboveCrossLine
        .and(!inAllianceZone)
        .onTrue(runOnce({ currentGoal = DEPOT_LOCATION }))

private val goalOutpostTrigger =
    outOfAllianceZoneBelowCrossLine
        .and(!inAllianceZone)
        .onTrue(runOnce({ currentGoal = OUTPOST_LOCATION }))

enum class ShootingType {
    STATIC,
    INTERPOLATION,
    SHOOT_ON_MOVE
}

val shootingType
    get() =
        when {
            DriverOverrides.StaticShootingOverride.trigger.asBoolean ->
                ShootingType.STATIC
            !DriverOverrides.ShootOnMoveOverride.trigger.asBoolean ->
                ShootingType.INTERPOLATION
            else -> ShootingType.SHOOT_ON_MOVE
        }

fun <T : SubsystemBase, M : Measure<out Unit>> T.aimingSetpoint(): M {
    val result =
        when (shootingType) {
            ShootingType.STATIC -> staticShootingMap[this]!!
            ShootingType.INTERPOLATION -> interpolationShootingMap[this]!!
            else -> shootOnMoveMap[this]!!
        }

    @Suppress("UNCHECKED_CAST") return result as M
}
