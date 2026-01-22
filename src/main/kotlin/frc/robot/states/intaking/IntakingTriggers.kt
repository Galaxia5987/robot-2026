package frc.robot.states.intaking

class IntakingTriggers {
    val intakingTrigger = IntakingStates.INTAKING.trigger.onTrue(intaking())

    val closedTrigger = IntakingStates.CLOSED.trigger.onTrue(closed())

    val openTrigger = IntakingStates.OPEN.trigger.onTrue(open())

    val plummingTrigger = IntakingStates.PUMPING.trigger.onTrue(pumping())
}
