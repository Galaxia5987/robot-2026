package frc.robot.states.spindexer

import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.subsystems.spindexer.Spindexer
import org.team5987.annotation.LogLevel
import org.team5987.annotation.LoggedOutput

private val stopTrigger = SpindexerStates.IDLE.trigger.onTrue(Spindexer.stop())
private val activeTrigger =
    SpindexerStates.ACTIVE.trigger.onTrue(Spindexer.start())

@LoggedOutput(LogLevel.COMP)
val spinRequested =
    Trigger { isFeeding || isIntaking }
        .onTrue(Commands.runOnce(SpindexerStates.ACTIVE::set))
        .onFalse(Commands.runOnce(SpindexerStates.IDLE::set))
