package frc.robot.states.spindexer

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.button.Trigger
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

val spinRequested= Trigger{ isFeeding || isIntaking }
    .onTrue(Commands.runOnce({SpindexerStates.SPIN.set()}))
    .onFalse(Commands.runOnce({SpindexerStates.IDLE.set()}))

