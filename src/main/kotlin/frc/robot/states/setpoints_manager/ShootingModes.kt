package frc.robot.states.setpoints_manager

import edu.wpi.first.units.Measure
import edu.wpi.first.units.Unit
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.states.setpoints_manager.SetpointsManager.shootingType
import frc.robot.states.setpoints_manager.shooting_modes.calibrationShootingMap
import frc.robot.states.setpoints_manager.shooting_modes.feedShootingMap
import frc.robot.states.setpoints_manager.shooting_modes.interpolationShootingMap
import frc.robot.states.setpoints_manager.shooting_modes.shootOnMoveMap
import frc.robot.states.setpoints_manager.shooting_modes.staticShootingMap

enum class ShootingType {
    STATIC,
    INTERPOLATION,
    SHOOT_ON_MOVE,
    CALIBRATION,
    FEEDING
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
