package frc.robot.autonomous

import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.events.EventTrigger
import com.pathplanner.lib.path.PathPlannerPath
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import frc.robot.lib.BetterPoseEstimator
import frc.robot.lib.extensions.flipIfNeeded
import frc.robot.lib.extensions.sec
import frc.robot.states.intaking.IntakingStates
import frc.robot.states.shooting.ShootingState
import frc.robot.states.shooting.setShootInAuto
import frc.robot.states.shooting.stopShootInAuto

private fun runPath(path: String) =
    AutoBuilder.followPath(PathPlannerPath.fromPathFile(path))

private fun runPathAndReset(path: String): Command {
    val path = PathPlannerPath.fromPathFile(path)
    val startPose = path.startingHolonomicPose.get()
    return Commands.runOnce({
            BetterPoseEstimator.getInstance()
                .resetPose(startPose.flipIfNeeded())
        })
        .andThen(AutoBuilder.followPath(path))
}

// TODO: After basic trajectory following is working open path planner and look at the paths then
// implement here the trajectory chaining as one command with state machine integrated commands as
// well
val setIntaking =
    EventTrigger("setIntaking").onTrue(IntakingStates.INTAKING.set())
val setStopIntaking =
    EventTrigger("setStopIntaking").onTrue(IntakingStates.CLOSED.set())
val setShooting =
    EventTrigger("setShooting").onTrue(ShootingState.SHOOTING.set())
val setStopShooting =
    EventTrigger("setStopShooting").onTrue(ShootingState.IDLE.set())

fun shootOnMoveTestPath(): Command =
    AutoBuilder.followPath(PathPlannerPath.fromPathFile("Test"))

fun test() {}

fun depotDoubleCycle(): Command =
    Commands.sequence(
        runPathAndReset("StartToFuelDepotSide"),
        runPath("FuelToDepotStart").alongWith(setShootInAuto()),
        Commands.waitTime(8.sec),
        stopShootInAuto(),
    )

fun depotMain(): Command =
    Commands.sequence(
        runPathAndReset("StartToFuelDepotSide"),
        runPath("FuelDepotSideToDepot"),
        setShootInAuto(),
        Commands.waitTime(10.sec),
        IntakingStates.INTAKING.set(),
        Commands.waitTime(0.5.sec),
        runPath("PickupDepot"),
        runPath("ExitDepot")
    )
