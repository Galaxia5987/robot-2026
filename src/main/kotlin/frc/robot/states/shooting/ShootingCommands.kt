package frc.robot.states.shooting

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import frc.robot.drive
import frc.robot.subsystems.shooter.flywheel.Flywheel
import frc.robot.subsystems.shooter.pre_shooter.PreShooter
import frc.robot.subsystems.spindexer.Spindexer

fun enableDontShoot() {
    dontShoot = true
}

fun disableDontShoot() {
    dontShoot = false
}

fun primeSubsystems() =
    Commands.sequence(
        Flywheel.setVelocity { Flywheel.aimingSetpoint() },
        PreShooter.setVelocity { Flywheel.aimingSetpoint() }
    )

// State Commands
fun idle(): Command =
    Commands.sequence(
        Flywheel.zero(),
        Spindexer.stop(), // TODO: Should probably use SpindexerManager
    ) // TODO: Close intake using intake state machine

fun priming(): Command = Commands.sequence(drive.lock(), primeSubsystems())

fun backfeeding(): Command =
    Commands.sequence(PreShooter.reverse(), Spindexer.reverse())

fun shooting(): Command =
    Commands.sequence(
        Spindexer.start(), // TODO: ditto
        // TODO: set intake state machine to pump
        )
