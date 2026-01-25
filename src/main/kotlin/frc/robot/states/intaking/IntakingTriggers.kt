package frc.robot.states.intaking

import frc.robot.states.sensors.Sensors.isHalfFull

private val intakingTrigger = IntakingStates.INTAKING.trigger.onTrue(intaking())

private val closedTrigger = IntakingStates.CLOSED.trigger.onTrue(closed())

private val openTrigger = IntakingStates.OPEN.trigger.onTrue(open())

private val pumpingTrigger = IntakingStates.PUMPING.trigger.onTrue(pumping())

val canCloseIntake = isHalfFull

val cantCloseIntake = isHalfFull.negate()
