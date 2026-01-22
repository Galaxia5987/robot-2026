package frc.robot.states.intaking

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import frc.robot.lib.extensions.sec
import frc.robot.roller
import frc.robot.subsystems.intake.extender.Extender
import frc.robot.subsystems.intake.extender.ExtenderPositions
import frc.robot.subsystems.roller.RollerPositions
import kotlin.concurrent.timerTask
import kotlin.system.measureTimeMillis
import kotlin.time.Duration.Companion.microseconds
import kotlin.time.Duration.Companion.milliseconds
import kotlin.time.Duration.Companion.seconds


fun closed(): Command{
    return roller.setTarget(RollerPositions.STOP).alongWith(Extender.setTarget(ExtenderPositions.CLOSE))
}

fun intaking(): Command{
    return roller.intake()
}

fun open(): Command{
    return Extender.setTarget(ExtenderPositions.OPEN)
}

fun pluming(): Command {
     return run {   Extender.setTarget(ExtenderPositions.OPEN)
    Commands.waitTime(0.4.sec).andThen(Extender.setTarget(ExtenderPositions.CLOSE))
     }
}
