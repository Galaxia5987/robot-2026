package frc.robot.states.intaking

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import frc.robot.lib.extensions.sec
import frc.robot.states.spindexer.SpindexerCommands
import frc.robot.subsystems.intake.extender.Extender
import frc.robot.subsystems.intake.extender.ExtenderPositions
import frc.robot.subsystems.intake.roller.Roller

fun closed(): Command {
    return Roller.stop().alongWith(Extender.close())
}

fun intaking(): Command {
    return Roller.intake()
        .alongWith(
            Extender.open(),
            (SpindexerCommands.startIntaking())
        ) // Spindexer Intaking
}

fun open(): Command {
    return Extender.open()
        .alongWith(Roller.intake(), SpindexerCommands.stopIntaking())
}

fun pumping(): Command {
    return Commands.sequence(
            Extender.setTarget(ExtenderPositions.OPEN),
            Commands.waitTime(0.4.sec),
            (Extender.setTarget(ExtenderPositions.CLOSE))
        )
        .repeatedly()
}
