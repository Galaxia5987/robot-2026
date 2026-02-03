package frc.robot.states.intaking

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import frc.robot.lib.extensions.sec
import frc.robot.states.spindexer.SpindexerCommands
import frc.robot.subsystems.intake.extender.Extender
import frc.robot.subsystems.intake.extender.ExtenderPositions
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

fun open(): Command =
    Commands.parallel(
        Extender.open(),
        Roller.stop(),
        SpindexerCommands.stopIntaking()
    )

fun pumping(): Command {
    return Commands.sequence(
            Extender.setTarget(ExtenderPositions.CLOSE),
            Commands.waitTime(PUMP_TIME),
            Extender.setTarget(ExtenderPositions.OPEN),
            Commands.waitTime(PUMP_TIME),
        )
        .repeatedly()
}
