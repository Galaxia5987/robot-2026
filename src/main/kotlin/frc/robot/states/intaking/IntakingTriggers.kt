package frc.robot.states.intaking

import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.RobotContainer
import frc.robot.drive

class IntakingTriggers {
    val intakingTrigger= IntakingStates.INTAKING.IntakingStatesTrigger.whileTrue(intaking())

    val closedTrigger= IntakingStates.CLOSED.IntakingStatesTrigger.whileTrue(closed())
}