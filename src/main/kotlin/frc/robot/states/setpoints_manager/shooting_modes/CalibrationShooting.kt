package frc.robot.states.setpoints_manager.shooting_modes

import edu.wpi.first.units.Measure
import edu.wpi.first.units.Unit
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.ShotCalculator.calculatePitch
import frc.robot.drive
import frc.robot.lib.extensions.deg
import frc.robot.lib.extensions.get
import frc.robot.lib.extensions.m
import frc.robot.lib.extensions.rps
import frc.robot.states.setpoints_manager.SetpointsManager
import frc.robot.states.setpoints_manager.SetpointsManager.turretDistanceFromGoal
import frc.robot.subsystems.shooter.flywheel.Flywheel
import frc.robot.subsystems.shooter.flywheel.Flywheel.calibrationVelocity
import frc.robot.subsystems.shooter.hood.Hood
import frc.robot.subsystems.shooter.pre_shooter.PreShooter
import frc.robot.subsystems.shooter.pre_shooter.PreShooterVelocity
import frc.robot.subsystems.shooter.turret.Turret
import kotlin.collections.mapOf

private fun getTurretSetpoint(): Angle = SetpointsManager.turretAngleToHub

private fun getHoodSetpoint(): Angle =
    (90.deg -
        calculatePitch(
                turretDistanceFromGoal[m],
                drive.chassisSpeeds.vxMetersPerSecond,
                drive.chassisSpeeds.vyMetersPerSecond
            )
            .deg)

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
        Turret to ::getTurretSetpoint,
        Hood to ::getHoodSetpoint,
        Flywheel to ::getFlywheelSetpoint,
        PreShooter to ::getPreShooterSetpoint,
    )
