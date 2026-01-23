package frc.robot.states.setpoints_manager.setpoints_state

import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.AngularVelocity
import frc.robot.subsystems.shooter.flywheel.Flywheel
import frc.robot.subsystems.shooter.hood.Hood
import frc.robot.subsystems.shooter.pre_shooter.PreShooter
import frc.robot.subsystems.shooter.turret.Turret

val interpolationShootingTurretSetpoint: Angle
    get() = TODO()
val interpolationShootingHoodSetpoint: Angle
    get() = TODO()
val interpolationShootingFlywheelSetpoint: AngularVelocity
    get() = TODO()
val interpolationShootingPreShooterSetpoint : AngularVelocity
    get() = TODO()

val interpolationShootingMap = mapOf(
    Turret to interpolationShootingTurretSetpoint,
    Hood to interpolationShootingHoodSetpoint,
    Flywheel to interpolationShootingFlywheelSetpoint,
    PreShooter to interpolationShootingPreShooterSetpoint
)