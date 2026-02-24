package frc.robot.autonomous

import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.commands.PathPlannerAuto
import com.pathplanner.lib.events.EventTrigger
import com.pathplanner.lib.path.PathPlannerPath
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import frc.robot.states.intaking.IntakingStates
import frc.robot.states.shooting.ShootingState

private fun runPath(path: String) = AutoBuilder.followPath(PathPlannerPath.fromPathFile(path))

// TODO: After basic trajectory following is working open path planner and look at the paths then
// implement here the trajectory chaining as one command with state machine integrated commands as
// well
val setIntaking= EventTrigger("setIntaking").onTrue(IntakingStates.INTAKING.set())
val setStopIntaking= EventTrigger("setStopIntaking").onTrue(IntakingStates.CLOSED.set())
val setShooting = EventTrigger("setShooting").onTrue(ShootingState.SHOOTING.set())
val setStopShooting= EventTrigger("setStopShooting").onTrue(ShootingState.IDLE.set())

fun shootOnMoveTestPath(): Command =
    AutoBuilder.followPath(PathPlannerPath.fromPathFile("Test"))

fun depotMain(): Command =
    Commands.sequence(runPath("StartToFuelDepotSide"), runPath("FuelToDepotStart"))
