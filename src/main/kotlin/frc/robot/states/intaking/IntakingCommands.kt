package frc.robot.states.intaking

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import frc.robot.lib.extensions.sec
import frc.robot.states.spindexer.SpindexerCommands
import frc.robot.subsystems.intake.extender.Extender
import frc.robot.subsystems.intake.roller.Roller

private val PUMP_TIME = 0.3.sec

fun closed(): Command =
    Commands.parallel(
        Roller.stop(),
        Extender.close(),
        SpindexerCommands.stopIntaking()
    )

fun intaking(): Command =
    Commands.parallel(
        Roller.intake(),
        Extender.open(),
        SpindexerCommands.startIntaking()
    )

fun pumping(): Command {
    val lastStallPoint = Extender.lastStallingDistance
    return Commands.sequence(
            Extender.close(),
            Extender.setPosition(lastStallPoint),
            Commands.waitTime(PUMP_TIME),
        )
        .repeatedly()
}
