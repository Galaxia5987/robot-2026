package frc.robot.states.intaking

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import frc.robot.lib.extensions.sec
import frc.robot.roller
import frc.robot.spindexer
import frc.robot.subsystems.intake.extender.Extender
import frc.robot.subsystems.intake.extender.ExtenderPositions
import frc.robot.subsystems.spindexer.Spindexer
import frc.robot.subsystems.spindexer.SpindexerVelocity

internal fun closed(): Command {
    return roller.stop().alongWith(Extender.close())
}

internal fun intaking(): Command {
    return roller
        .intake()
        .alongWith(
            Extender.open()
                .alongWith(Spindexer.setTarget(SpindexerVelocity.START))
        ) // Spindexer Intaking
}

internal fun open(): Command {
    return Extender.open()
        .alongWith(spindexer.setTarget(SpindexerVelocity.STOP))
}

internal fun pumping(): Command {
    return Commands.sequence(
            Extender.setTarget(ExtenderPositions.OPEN),
            Commands.waitTime(0.4.sec),
            (Extender.setTarget(ExtenderPositions.CLOSE))
        )
        .repeatedly()
}
