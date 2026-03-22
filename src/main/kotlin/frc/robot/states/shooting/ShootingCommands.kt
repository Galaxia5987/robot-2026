package frc.robot.states.shooting

import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.RobotContainer
import frc.robot.lib.extensions.logTrigger
import frc.robot.states.intaking.IntakingStates
import frc.robot.states.setpoints_manager.aimingSetpoint
import frc.robot.states.spindexer.LOGGING_PATH
import frc.robot.states.spindexer.SpindexerCommands
import frc.robot.subsystems.shooter.flywheel.Flywheel
import frc.robot.subsystems.shooter.pre_shooter.PreShooter

val shouldPump: Trigger =
    IntakingStates.INTAKING.trigger
        .negate()
        .and(ShootingState.SHOOTING.trigger)
        .and(Trigger({RobotContainer.shooting?.canDoubleFeed?.negate()?.asBoolean ?: true}))
        .whileTrue(IntakingStates.PUMPING.set())
        .logTrigger("$LOGGING_PATH/shouldPump")

// State Commands
fun idle(): Command =
    Commands.sequence(
        Flywheel.zero(),
        PreShooter.stop(),
        SpindexerCommands.stopFeeding(),
        IntakingStates.CLOSED.set()
            .onlyIf(IntakingStates.INTAKING.trigger.negate())
    )

fun priming(): Command =
    Flywheel.setVelocity(
        Flywheel.aimingSetpoint<Flywheel, () -> AngularVelocity>()
    )
        .alongWith(
            PreShooter.setVelocity(
                PreShooter.aimingSetpoint<PreShooter, () -> AngularVelocity>()
            )
        )

fun backfeeding(): Command = PreShooter.reverse()

fun shooting(): Command =
    Commands.sequence(
        SpindexerCommands.startFeeding(),
        //            IntakingStates.PUMPING.set().onlyIf(shouldPump)
    )
        .alongWith(priming())