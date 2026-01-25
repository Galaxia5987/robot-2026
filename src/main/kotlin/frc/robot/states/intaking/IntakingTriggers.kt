package frc.robot.states.intaking

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.states.sensors.Sensors.isHalfFull

private val intakingTrigger = IntakingStates.INTAKING.trigger.onTrue(intaking())

private val closedTrigger = IntakingStates.CLOSED.trigger.onTrue(closed())

private val openTrigger = IntakingStates.OPEN.trigger.onTrue(open())

private val pumpingTrigger = IntakingStates.PUMPING.trigger.onTrue(pumping())

fun canClose(): Command {
    return if (isHalfFull.asBoolean == false) {
        IntakingStates.CLOSED.set()
    } else {
        IntakingStates.OPEN.set()
    }
}
