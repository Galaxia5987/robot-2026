package frc.robot.states.setpoints_manager

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.units.Measure
import edu.wpi.first.units.Unit
import edu.wpi.first.wpilibj2.command.Commands.runOnce
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.field.DEPOT_LOCATION
import frc.robot.field.HUB_LOCATION
import frc.robot.field.OUTPOST_LOCATION
import frc.robot.field.inAllianceZone
import frc.robot.field.isCloserToDepotSide
import frc.robot.lib.extensions.not
import frc.robot.lib.extensions.toPose
import frc.robot.states.DriverOverrides
import frc.robot.states.setpoints_manager.shooting_modes.interpolationShootingMap
import frc.robot.states.setpoints_manager.shooting_modes.shootOnMoveMap
import frc.robot.states.setpoints_manager.shooting_modes.staticShootingMap
import org.team5987.annotation.LogLevel
import org.team5987.annotation.LoggedOutput

@LoggedOutput(key = "/StateMachines/Shooting/currentGoal", level = LogLevel.COMP) var currentGoal: Pose2d = HUB_LOCATION.toPose()

private val goalHubTrigger =
    inAllianceZone.onTrue(runOnce({ currentGoal = HUB_LOCATION.toPose() }))

private val goalDepotTrigger =
    isCloserToDepotSide
        .and(!inAllianceZone)
        .onTrue(runOnce({ currentGoal = DEPOT_LOCATION.toPose() }))

private val goalOutpostTrigger =
    isCloserToDepotSide
        .negate()
        .and(!inAllianceZone)
        .onTrue(runOnce({ currentGoal = OUTPOST_LOCATION.toPose() }))

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

val isShootingOnMove = Trigger { shootingType == ShootingType.SHOOT_ON_MOVE }

val isUsingInterpolation = Trigger {
    shootingType == ShootingType.INTERPOLATION
}

val isUsingStaticSetpoints = Trigger { shootingType == ShootingType.STATIC }

fun <T : SubsystemBase, M : Measure<out Unit>> T.aimingSetpoint(): M {
    val result =
        when (shootingType) {
            ShootingType.STATIC -> staticShootingMap[this]!!
            ShootingType.INTERPOLATION -> interpolationShootingMap[this]!!
            else -> shootOnMoveMap[this]!!
        }

    @Suppress("UNCHECKED_CAST") return result.invoke() as M
}
