package frc.robot.states.setpoints_manager.setpoints_state

import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.AngularVelocity
import frc.robot.subsystems.shooter.flywheel.Flywheel
import frc.robot.subsystems.shooter.hood.Hood
import frc.robot.subsystems.shooter.pre_shooter.PreShooter
import frc.robot.subsystems.shooter.turret.Turret

val staticShootingTurretSetpoint: Angle
    get() = TODO()
val staticShootingHoodSetpoint: Angle
    get() = TODO()
val staticShootingFlywheelSetpoint: AngularVelocity
    get() = TODO()
val staticShootingPreShooterSetpoint : AngularVelocity
    get() = TODO()

val staticShootingMap = mapOf(
    Turret to staticShootingTurretSetpoint,
    Hood to staticShootingHoodSetpoint,
    Flywheel to staticShootingFlywheelSetpoint,
    PreShooter to staticShootingPreShooterSetpoint
)