package frc.robot.states.setpoints_manager

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.units.Measure
import edu.wpi.first.units.Unit
import edu.wpi.first.wpilibj2.command.Commands.runOnce
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.field.*
import frc.robot.lib.extensions.not
import frc.robot.lib.extensions.toPose
import frc.robot.states.DriverOverrides
import frc.robot.states.setpoints_manager.SetpointsManager.ShootingType
import frc.robot.states.setpoints_manager.SetpointsManager.shootingType
import frc.robot.states.setpoints_manager.shooting_modes.calibrationShootingMap
import frc.robot.states.setpoints_manager.shooting_modes.interpolationShootingMap
import frc.robot.states.setpoints_manager.shooting_modes.shootOnMoveMap
import frc.robot.states.setpoints_manager.shooting_modes.staticShootingMap
import org.team5987.annotation.LogLevel
import org.team5987.annotation.LoggedOutput

object SetpointsManager {

    @LoggedOutput(
        key = "StateMachines/Shooting/currentGoal",
        level = LogLevel.COMP
    )
    var currentGoal: Pose2d = HUB_TRANSLATION.toPose()

    private val goalHubTrigger =
        inAllianceZone.onTrue(
            runOnce({ currentGoal = HUB_TRANSLATION.toPose() })
                .ignoringDisable(true)
        )

    private val goalDepotTrigger =
        isCloserToDepotSide
            .and(!inAllianceZone)
            .onTrue(
                runOnce({ currentGoal = DEPOT_TRANSLATION.toPose() })
                    .ignoringDisable(true)
            )

    private val goalOutpostTrigger =
        isCloserToDepotSide
            .negate()
            .and(!inAllianceZone)
            .onTrue(
                runOnce({ currentGoal = OUTPOST_LOCATION.toPose() })
                    .ignoringDisable(true)
            )

    enum class ShootingType {
        STATIC,
        INTERPOLATION,
        SHOOT_ON_MOVE,
        CALIBRATION
    }

    @LoggedOutput(
        key = "StateMachines/Shooting/shootingType",
        level = LogLevel.COMP
    )
    val shootingType
        get() =
            when {
                DriverOverrides.StaticShootingOverride.trigger.asBoolean ->
                    ShootingType.STATIC
                DriverOverrides.ShootingCalibrationOverride.trigger.asBoolean ->
                    ShootingType.CALIBRATION
                DriverOverrides.ShootOnMoveOverride.trigger.asBoolean ->
                    ShootingType.SHOOT_ON_MOVE
                else -> ShootingType.INTERPOLATION
            }

    val isShootingOnMove = Trigger {
        shootingType == ShootingType.SHOOT_ON_MOVE
    }

    val isUsingInterpolation = Trigger {
        shootingType == ShootingType.INTERPOLATION
    }

    val isUsingStaticSetpoints = Trigger { shootingType == ShootingType.STATIC }
}

fun <T : SubsystemBase, M : () -> Measure<out Unit>> T.aimingSetpoint(): M {
    @Suppress("UNCHECKED_CAST")
    return {
        val result =
            when (shootingType) {
                ShootingType.STATIC -> staticShootingMap[this]!!
                ShootingType.INTERPOLATION -> interpolationShootingMap[this]!!
                ShootingType.CALIBRATION -> calibrationShootingMap[this]!!
                else -> shootOnMoveMap[this]!!
            }
        result.invoke()
    }
        as M
}
