package frc.robot.states.intaking

import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.states.sensors.isHalfFull
import frc.robot.subsystems.intake.extender.Extender

    val intakingTrigger = IntakingStates.INTAKING.trigger.onTrue(intaking())

    val closedTrigger = IntakingStates.CLOSED.trigger.onTrue(closed())

    val openTrigger = IntakingStates.OPEN.trigger.onTrue(open())

    val plummingTrigger = IntakingStates.PUMPING.trigger.onTrue(pumping())

    val canClose= isHalfFull.onFalse(closed())




