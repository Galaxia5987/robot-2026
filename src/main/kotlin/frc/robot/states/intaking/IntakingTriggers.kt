package frc.robot.states.intaking

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.spindexer
import frc.robot.states.sensors.isHalfFull
import frc.robot.subsystems.intake.extender.Extender
import frc.robot.subsystems.spindexer.SpindexerVelocity

val intakingTrigger = IntakingStates.INTAKING.trigger.onTrue(intaking())

    val closedTrigger = IntakingStates.CLOSED.trigger.onTrue(closed())

    val openTrigger = IntakingStates.OPEN.trigger.onTrue(open())

    val plummingTrigger = IntakingStates.PUMPING.trigger.onTrue(pumping())

fun canClose(): Command{
    return if (isHalfFull.asBoolean==false){
        closed()
    }else{
        open()
    }
}


