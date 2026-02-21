package frc.robot.states.intaking

import frc.robot.lib.extensions.onTrue

object IntakingTriggers {
    private val isClosed = IntakingStates.CLOSED.trigger.onTrue(closed())
    private val isIntaking = IntakingStates.INTAKING.trigger.onTrue(intaking())
    private val isPumping = IntakingStates.PUMPING.trigger.whileTrue(pumping())
}
