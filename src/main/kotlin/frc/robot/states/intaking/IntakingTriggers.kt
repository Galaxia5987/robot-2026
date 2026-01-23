package frc.robot.states.intaking

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.states.sensors.isHalfFull

val intakingTrigger = IntakingStates.INTAKING.trigger.onTrue(intaking())

val closedTrigger = IntakingStates.CLOSED.trigger.onTrue(closed())

val openTrigger = IntakingStates.OPEN.trigger.onTrue(open())

val plummingTrigger = IntakingStates.PUMPING.trigger.onTrue(pumping())

fun canClose(): Command {
    return if (isHalfFull.asBoolean == false) {
        closed()
    } else {
        open()
    }
}
