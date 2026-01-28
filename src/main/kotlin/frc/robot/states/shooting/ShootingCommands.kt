package frc.robot.states.shooting

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import frc.robot.states.intaking.IntakingStates
import frc.robot.states.setpoints_manager.aimingSetpoint
import frc.robot.states.spindexer.SpindexerCommands
import frc.robot.subsystems.shooter.flywheel.Flywheel
import frc.robot.subsystems.shooter.pre_shooter.PreShooter

// State Commands
internal fun idle(): Command =
    Commands.sequence(
        Flywheel.zero(),
        SpindexerCommands.stopFeeding(),
        IntakingStates.CLOSED.set()
    )

internal fun priming(): Command =
    Commands.sequence(
        Flywheel.setVelocity { Flywheel.aimingSetpoint() },
        PreShooter.setVelocity { Flywheel.aimingSetpoint() }
    )

internal fun backfeeding(): Command = PreShooter.reverse()

internal fun shooting(): Command =
    Commands.sequence(
        SpindexerCommands.startFeeding(),
        IntakingStates.PUMPING.set()
    )
