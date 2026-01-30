package frc.robot.states.intaking

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import frc.robot.lib.extensions.sec
import frc.robot.states.spindexer.SpindexerCommands
import frc.robot.subsystems.intake.extender.Extender
import frc.robot.subsystems.intake.extender.ExtenderPositions
import frc.robot.subsystems.intake.roller.Roller

fun closed(): Command = Commands.parallel(Roller.stop(), Extender.close(), SpindexerCommands.stopIntaking())

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

fun pumping(): Command =
    Commands.sequence(
            Extender.setTarget(ExtenderPositions.OPEN),
            Commands.waitTime(0.4.sec),
            Extender.setTarget(ExtenderPositions.CLOSE)
        )
        .repeatedly()
