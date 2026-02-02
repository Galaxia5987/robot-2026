package frc.robot.states.spindexer

import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.lib.extensions.logTrigger
import frc.robot.states.spindexer.SpindexerCommands.isFeeding
import frc.robot.states.spindexer.SpindexerCommands.isIntaking
import frc.robot.subsystems.spindexer.Spindexer
import org.team5987.annotation.LogLevel
import org.team5987.annotation.LoggedOutput

const val LOGGING_PATH = "StateMachines/Spindexer"

private val stopTrigger = SpindexerStates.IDLE.trigger.onTrue(Spindexer.stop())
private val activeTrigger =
    SpindexerStates.ACTIVE.trigger.onTrue(Spindexer.start())

val spinRequested: Trigger =
    Trigger { isFeeding || isIntaking }
        .onTrue(SpindexerStates.ACTIVE.set())
        .onFalse(SpindexerStates.IDLE.set()).logTrigger("$LOGGING_PATH/spinRequested")
