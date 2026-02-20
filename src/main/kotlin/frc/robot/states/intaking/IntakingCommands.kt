package frc.robot.states.intaking

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import frc.robot.lib.extensions.cm
import frc.robot.lib.extensions.sec
import frc.robot.states.spindexer.SpindexerCommands
import frc.robot.subsystems.intake.extender.Extender
import frc.robot.subsystems.intake.roller.Roller

private val PUMP_TIME = 0.3.sec

fun closed(): Command =
    Commands.parallel(
        Roller.stop(),
        Extender.close(),
    )

fun intaking(): Command =
    Commands.parallel(
        Roller.intake(),
        Extender.open(),
    )

fun pumping(): Command =
    Commands.repeatingSequence(
        Extender.defer {
            Commands.sequence(
                Extender.close(),
                Commands.waitUntil(Extender.shouldStop),
                Extender.defer {
                    Extender.setPosition(Extender.lastStallingDistance + 10.cm)
                },
                Commands.waitTime(PUMP_TIME),
            )
        }
    )
