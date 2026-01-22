package frc.robot.states.spindexer

import frc.robot.spindexer
import frc.robot.subsystems.spindexer.ConveyorVelocity

val spinState =
    SpindexerStates.SPIN.trigger.onTrue(
        spindexer.setTarget(ConveyorVelocity.START)
    )
val idleState =
    SpindexerStates.IDLE.trigger.onTrue(
        spindexer.setTarget(ConveyorVelocity.STOP)
    )



