package frc.robot.autonomous

import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.commands.PathPlannerAuto
import com.pathplanner.lib.events.EventTrigger
import com.pathplanner.lib.path.PathPlannerPath
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.states.intaking.IntakingStates

// TODO: After basic trajectory following is working open path planner and look at the paths then
// implement here the trajectory chaining as one command with state machine integrated commands as
// well
val setIntaking =
    EventTrigger("setIntaking").onTrue(IntakingStates.INTAKING.set())
val setStopIntaking =
    EventTrigger("setStopIntaking").onTrue(IntakingStates.CLOSED.set())

fun DepotMain(): Command = PathPlannerAuto("DepotMain")

fun AutoTest(): Command = PathPlannerAuto("AutoTest")

fun Test(): Command =
    AutoBuilder.resetOdom(PathPlannerPath.fromPathFile("Test").pathPoses[0])
        .andThen(AutoBuilder.followPath(PathPlannerPath.fromPathFile("Test")))

fun AutoTestCommands(): Command = PathPlannerAuto("AutoTestCommands")

fun TwoPathTestAuto(): Command = PathPlannerAuto("TwoPathTestAuto")

fun shootOnMoveTestPath(): Command =
    AutoBuilder.followPath(PathPlannerPath.fromPathFile("Test"))
