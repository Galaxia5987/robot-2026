package frc.robot.states.intaking

import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.subsystems.sensors.Sensors.isHalfFull

object IntakingTriggers {

    val canCloseIntake: Trigger = isHalfFull.negate()

    val cantCloseIntake = isHalfFull

    private val isClosed = IntakingStates.CLOSED.trigger.onTrue(closed())
    private val isIntaking = IntakingStates.INTAKING.trigger.onTrue(intaking())
    private val isOpen = IntakingStates.OPEN.trigger.onTrue(open())
    private val isPumping = IntakingStates.PUMPING.trigger.onTrue(pumping())
}
