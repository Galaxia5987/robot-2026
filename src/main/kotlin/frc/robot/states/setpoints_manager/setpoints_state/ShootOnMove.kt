package frc.robot.states.setpoints_manager.setpoints_state

import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.AngularVelocity
import frc.robot.subsystems.shooter.flywheel.Flywheel
import frc.robot.subsystems.shooter.hood.Hood
import frc.robot.subsystems.shooter.pre_shooter.PreShooter
import frc.robot.subsystems.shooter.turret.Turret

val shootOnMoveTurretSetpoint: Angle
    get() = TODO()
val shootOnMoveHoodSetpoint: Angle
    get() = TODO()
val shootOnMoveFlywheelSetpoint: AngularVelocity
    get() = TODO()
val shootOnMovePreShooterSetpoint : AngularVelocity
    get() = TODO()

val shootOnMoveMap = mapOf(
    Turret to shootOnMoveTurretSetpoint,
    Hood to shootOnMoveHoodSetpoint,
    Flywheel to shootOnMoveFlywheelSetpoint,
    PreShooter to shootOnMovePreShooterSetpoint
)