package frc.robot.states.setpoints_manager

import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.units.Measure
import edu.wpi.first.units.Unit
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.field.DEPOT_LOCATION
import frc.robot.field.HUB_LOCATION
import frc.robot.field.OUTPOST_LOCATION
import frc.robot.field.inAllianceZone
import frc.robot.field.outOfAllianceZone_AboveCrossLine
import frc.robot.field.outOfAllianceZone_BelowCrossLine
import frc.robot.states.DriverOverrides
import frc.robot.states.setpoints_manager.setpoints_state.interpolationShootingMap
import frc.robot.states.setpoints_manager.setpoints_state.shootOnMoveMap
import frc.robot.states.setpoints_manager.setpoints_state.staticShootingMap
import frc.robot.subsystems.shooter.flywheel.Flywheel

private fun <T : SubsystemBase> getSetpointByOverrides(pose: Translation2d, subsystem: T): Measure<out Unit> {
    if (DriverOverrides.StaticShootingOverride.trigger.asBoolean) {
        return staticShootingMap[subsystem]!!
    }else if (!DriverOverrides.ShootOnMoveOverride.trigger.asBoolean) {
        return interpolationShootingMap[subsystem]!!
    }
    return shootOnMoveMap[subsystem]!!
}

fun <T : SubsystemBase, M : Measure<out Unit>> T.aimingSetpoint(): M {
    val target = when {
        inAllianceZone.asBoolean -> HUB_LOCATION
        outOfAllianceZone_AboveCrossLine.asBoolean -> DEPOT_LOCATION
        outOfAllianceZone_BelowCrossLine.asBoolean -> OUTPOST_LOCATION
        else -> Translation2d()
    }

    @Suppress("UNCHECKED_CAST")
    return getSetpointByOverrides(target, this) as M
}

fun testUsage() {
//    val test: Angle = Turret.aimingSetpoint()
    val flywheelVelocity: AngularVelocity = Flywheel.aimingSetpoint()
}

//Is it katzuv approved? 🔥