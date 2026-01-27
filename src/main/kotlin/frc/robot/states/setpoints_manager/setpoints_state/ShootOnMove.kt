package frc.robot.states.setpoints_manager.setpoints_state

import edu.wpi.first.units.Measure
import edu.wpi.first.units.Unit
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.calculateLinearToAngularVelocity
import frc.robot.calculatePitch
import frc.robot.calculateVelocity
import frc.robot.calculateYaw
import frc.robot.drive
import frc.robot.lib.extensions.deg
import frc.robot.lib.extensions.get
import frc.robot.lib.extensions.m
import frc.robot.lib.extensions.rps
import frc.robot.subsystems.shooter.flywheel.Flywheel
import frc.robot.subsystems.shooter.hood.Hood
import frc.robot.subsystems.shooter.pre_shooter.PreShooter
import frc.robot.subsystems.shooter.turret.Turret

private fun getTurretSetpoint(): Angle {
    return calculateYaw(
            distanceFromGoal[m],
            drive.chassisSpeeds.vxMetersPerSecond,
            drive.chassisSpeeds.vyMetersPerSecond
        )
        .deg
}

private fun getHoodSetpoint(): Angle {
    return calculatePitch(
            distanceFromGoal[m],
            drive.chassisSpeeds.vxMetersPerSecond,
            drive.chassisSpeeds.vyMetersPerSecond
        )
        .deg
}

private fun getFlywheelSetpoint(): AngularVelocity {
    return calculateLinearToAngularVelocity(
            calculateVelocity(
                distanceFromGoal[m],
                drive.chassisSpeeds.vxMetersPerSecond,
                drive.chassisSpeeds.vyMetersPerSecond
            )
        )
        .rps
}

private fun getPreShooterSetpoint(): AngularVelocity {
    return calculateLinearToAngularVelocity(
            calculateVelocity(
                distanceFromGoal[m],
                drive.chassisSpeeds.vxMetersPerSecond,
                drive.chassisSpeeds.vyMetersPerSecond
            )
        )
        .rps
}

val shootOnMoveMap: Map<SubsystemBase, () -> Measure<out Unit>> =
    mapOf(
        Turret to ::getTurretSetpoint,
        Hood to ::getHoodSetpoint,
        Flywheel to ::getFlywheelSetpoint,
        PreShooter to ::getPreShooterSetpoint,
    )
