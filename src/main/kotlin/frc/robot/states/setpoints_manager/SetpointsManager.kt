package frc.robot.states.setpoints_manager

import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.units.Measure
import edu.wpi.first.units.Unit
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.wpilibj2.command.Commands.runOnce
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
import org.team5987.annotation.LogLevel
import org.team5987.annotation.LoggedOutput

enum class FieldLocation(location: Translation2d) { //Flip when red
    HUB (Translation2d()),
    CLIMB (Translation2d()),
    UPPER_BUMPER (Translation2d()),
    LOWER_BUMPER (Translation2d()),
    UPPER_TRENCH (Translation2d()),
    LOWER_TRENCH (Translation2d()),
    UPPER_CORNER (Translation2d()),
    LOWER_CORNER (Translation2d());
}

@LoggedOutput(LogLevel.COMP)
var currentGoal: Translation2d = HUB_LOCATION

private val goalHubTrigger = inAllianceZone.onTrue(runOnce({
    currentGoal = HUB_LOCATION
}))

private val goalDepotTrigger = outOfAllianceZone_AboveCrossLine.and(inAllianceZone.negate()).onTrue(runOnce({
    currentGoal = DEPOT_LOCATION
}))

private val goalOutpostTrigger = outOfAllianceZone_BelowCrossLine.and(inAllianceZone.negate()).onTrue(runOnce({
    currentGoal = OUTPOST_LOCATION
}))


fun <T : SubsystemBase, M : Measure<out Unit>> T.aimingSetpoint(): M {
    val result = when {
        DriverOverrides.StaticShootingOverride.trigger.asBoolean ->
            staticShootingMap[this]!!

        !DriverOverrides.ShootOnMoveOverride.trigger.asBoolean ->
            interpolationShootingMap[this]!!

        else ->
            shootOnMoveMap[this]!!
    }

    @Suppress("UNCHECKED_CAST")
    return result as M
}


// Don't ask questions. do this:
fun testUsage() {
//    val test: Angle = Turret.aimingSetpoint()
    val flywheelVelocity: AngularVelocity = Flywheel.aimingSetpoint()
}

//Is it katzuv approved? 🔥