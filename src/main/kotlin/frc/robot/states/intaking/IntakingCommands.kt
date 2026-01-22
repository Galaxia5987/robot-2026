package frc.robot.states.intaking

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import frc.robot.lib.extensions.sec
import frc.robot.roller
import frc.robot.subsystems.intake.extender.Extender
import frc.robot.subsystems.intake.extender.ExtenderPositions

fun closed(): Command {
    return roller.stop().alongWith(Extender.close())
}

fun intaking(): Command {
    return roller.intake().alongWith(Extender.open()) // Spindexer Intaking
}

fun open(): Command {
    return Extender.open()
}

fun pumping(): Command {
    return Commands.sequence(Extender.setTarget(ExtenderPositions.OPEN),
        Commands.waitTime(0.4.sec),
        (Extender.setTarget(ExtenderPositions.CLOSE))) .repeatedly()
}

