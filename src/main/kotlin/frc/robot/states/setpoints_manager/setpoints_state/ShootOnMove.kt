package frc.robot.states.setpoints_manager.setpoints_state

import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.units.Measure
import edu.wpi.first.units.Unit
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.subsystems.shooter.flywheel.Flywheel
import frc.robot.subsystems.shooter.hood.Hood
import frc.robot.subsystems.shooter.pre_shooter.PreShooter
import frc.robot.subsystems.shooter.turret.Turret


private fun getTurretSetpoint(): Angle {
    return TODO()
}

private fun getHoodSetpoint(): Angle {
    return TODO()
}

private fun getFlywheelSetpoint(): AngularVelocity {
    return TODO()
}

private fun getPreShooterSetpoint(): AngularVelocity {
    return TODO()
}

val shootOnMoveMap: Map<SubsystemBase, () -> Measure<out Unit>> = mapOf(
    Turret to ::getTurretSetpoint,
    Hood to ::getHoodSetpoint,
    Flywheel to ::getFlywheelSetpoint,
    PreShooter to ::getPreShooterSetpoint,
)