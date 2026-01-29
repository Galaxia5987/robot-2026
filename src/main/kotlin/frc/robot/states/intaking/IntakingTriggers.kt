package frc.robot.states.intaking

import frc.robot.subsystems.sensors.Sensors.isHalfFull

val canCloseIntake = isHalfFull

val cantCloseIntake = isHalfFull.negate()

private val isClosed = IntakingStates.CLOSED.trigger.onTrue(closed())
private val isIntaking = IntakingStates.INTAKING.trigger.onTrue(intaking())
private val isOpen = IntakingStates.OPEN.trigger.onTrue(open())
private val isPumping = IntakingStates.PUMPING.trigger.onTrue(pumping())