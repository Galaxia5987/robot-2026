package frc.robot.states.shooting

import frc.robot.subsystems.shooter.flywheel.Flywheel
import frc.robot.subsystems.shooter.hood.Hood
import frc.robot.subsystems.shooter.pre_shooter.PreShooter
import frc.robot.subsystems.shooter.turret.Turret

private val atGoal =
    Hood.atSetpoint
        .and(Turret.atSetpoint)
        .and(Flywheel.atSetpoint)
        .and(PreShooter.atSetpoint)
