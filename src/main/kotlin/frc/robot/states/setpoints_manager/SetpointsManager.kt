package frc.robot.states.setpoints_manager

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.units.Measure
import edu.wpi.first.units.Unit
import edu.wpi.first.wpilibj2.command.Commands.runOnce
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.field.*
import frc.robot.isAuto
import frc.robot.lib.extensions.not
import frc.robot.lib.extensions.onFalse
import frc.robot.lib.extensions.onTrue
import frc.robot.lib.extensions.toPose
import frc.robot.states.DriverOverrides
import frc.robot.states.setpoints_manager.SetpointsManager.ShootingType
import frc.robot.states.setpoints_manager.SetpointsManager.shootingType
import frc.robot.states.setpoints_manager.shooting_modes.calibrationShootingMap
import frc.robot.states.setpoints_manager.shooting_modes.feedShootingMap
import frc.robot.states.setpoints_manager.shooting_modes.interpolationShootingMap
import frc.robot.states.setpoints_manager.shooting_modes.shootOnMoveMap
import frc.robot.states.setpoints_manager.shooting_modes.staticShootingMap
import frc.robot.subsystems.shooter.flywheel.Flywheel
import frc.robot.subsystems.shooter.pre_shooter.PreShooter
import frc.robot.subsystems.spindexer.Spindexer
import org.team5987.annotation.LogLevel
import org.team5987.annotation.LoggedOutput

object SetpointsManager {

    @LoggedOutput(
        key = "StateMachines/Shooting/currentGoal",
        level = LogLevel.COMP
    )
    var currentGoal: Pose2d = HUB_TRANSLATION.toPose()
    var isFeeding = false

    private val goalHubTrigger =
        inExtendedAllianceZone
            .or(isAuto)
            .onTrue(
                runOnce({
                        currentGoal = HUB_TRANSLATION.toPose()
                        isFeeding = false
                    })
                    .ignoringDisable(true)
            )

    private val goalDepotTrigger =
        isCloserToDepotSide
            .and(!inExtendedAllianceZone)
            .and(isAuto.negate())
            .onTrue(
                runOnce({
                        currentGoal = DEPOT_TRANSLATION.toPose()
                        isFeeding = true
                    })
                    .ignoringDisable(true)
            )

    private val goalOutpostTrigger =
        isCloserToDepotSide
            .negate()
            .and(!inExtendedAllianceZone)
            .and(isAuto.negate())
            .onTrue(
                runOnce({
                        currentGoal = OUTPOST_LOCATION.toPose()
                        isFeeding = true
                    })
                    .ignoringDisable(true)
            )

    enum class ShootingType {
        STATIC,
        INTERPOLATION,
        SHOOT_ON_MOVE,
        CALIBRATION,
        FEEDING
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
                isFeeding -> ShootingType.FEEDING
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
    val isUsingFeeding =
        Trigger { isFeeding }
            .onTrue(
                Flywheel.setLowCurrentLimits(),
                PreShooter.setLowCurrentLimits(),
            )
            .onFalse(
                Flywheel.setRegularCurrentLimits(),
                PreShooter.setLowCurrentLimits(),
            )
}

fun <T : SubsystemBase, M : () -> Measure<out Unit>> T.aimingSetpoint(): M {
    @Suppress("UNCHECKED_CAST")
    return {
        val result =
            when (shootingType) {
                ShootingType.STATIC -> staticShootingMap[this]!!
                ShootingType.INTERPOLATION -> interpolationShootingMap[this]!!
                ShootingType.CALIBRATION -> calibrationShootingMap[this]!!
                ShootingType.SHOOT_ON_MOVE -> shootOnMoveMap[this]!!
                ShootingType.FEEDING -> feedShootingMap[this]!!
            }
        result.invoke()
    }
        as M
}
