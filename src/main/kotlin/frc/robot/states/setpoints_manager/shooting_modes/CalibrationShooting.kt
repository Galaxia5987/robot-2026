package frc.robot.states.setpoints_manager.shooting_modes

import edu.wpi.first.units.Measure
import edu.wpi.first.units.Unit
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.drive
import frc.robot.lib.extensions.deg
import frc.robot.lib.extensions.get
import frc.robot.lib.extensions.m
import frc.robot.lib.extensions.rps
import frc.robot.subsystems.shooter.flywheel.Flywheel
import frc.robot.subsystems.shooter.flywheel.Flywheel.calibrationVelocity
import frc.robot.subsystems.shooter.pre_shooter.PreShooter
import frc.robot.subsystems.shooter.pre_shooter.PreShooterVelocity
import kotlin.collections.mapOf

private fun getFlywheelSetpoint(): AngularVelocity =
    calibrationVelocity.get().rps

private fun getPreShooterSetpoint(): AngularVelocity {
    return PreShooterVelocity.SHOOTING.velocity

    // Makes the preshooter velocity match the flywheel velocity
    //    val preShooterKey = InterpolatingDouble(distanceFromGoal[m])
    //    return SHOOTER_VELOCITY_BY_DISTANCE.getInterpolated(preShooterKey).value.rps
}

val calibrationShootingMap: Map<SubsystemBase, () -> Measure<out Unit>> =
    mapOf(
        Flywheel to ::getFlywheelSetpoint,
        PreShooter to ::getPreShooterSetpoint,
    )
