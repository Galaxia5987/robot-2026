package frc.robot.states.intaking

class IntakingTriggers {
    val intakingTrigger= IntakingStates.INTAKING.IntakingStatesTrigger.whileTrue(intaking())

    val closedTrigger= IntakingStates.CLOSED.IntakingStatesTrigger.whileTrue(closed())

    val openTrigger= IntakingStates.OPEN.IntakingStatesTrigger.whileTrue(open())

    val plummingTrigger= IntakingStates.PLUMPING.IntakingStatesTrigger.whileTrue(pluming())
}