package frc.robot.autonomous

import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.events.EventTrigger
import com.pathplanner.lib.path.PathPlannerPath
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import frc.robot.lib.BetterPoseEstimator
import frc.robot.lib.extensions.flipIfNeeded
import frc.robot.lib.extensions.mirror
import frc.robot.lib.extensions.sec
import frc.robot.states.intaking.IntakingStates
import frc.robot.states.shooting.ShootingState
import frc.robot.states.shooting.setShootInAuto
import frc.robot.states.shooting.stopShootInAuto

private fun runPath(path: String, mirror: Boolean = false): Command {
    var path = PathPlannerPath.fromPathFile(path)
    if (mirror) {
        path = path.mirrorPath()
    }
    return AutoBuilder.followPath(path)
}

private fun runPathAndReset(pathName: String, mirror: Boolean = false): Command {
    val path = PathPlannerPath.fromPathFile(pathName)
    var startPose = path.startingHolonomicPose.get()
    if (mirror) {
        startPose = startPose.mirror()
    }

    return Commands.runOnce({
        BetterPoseEstimator.getInstance()
            .resetPose(startPose.flipIfNeeded())
    })
        .andThen(runPath(pathName, mirror))
}

// TODO: After basic trajectory following is working open path planner and look at the paths then
// implement here the trajectory chaining as one command with state machine integrated commands as
// well
val setIntaking =
    EventTrigger("setIntaking").onTrue(IntakingStates.INTAKING.set())
val setStopIntaking =
    EventTrigger("setStopIntaking").onTrue(IntakingStates.CLOSED.set())
val setShooting = EventTrigger("setShooting").onTrue(setShootInAuto())
val setStopShooting = EventTrigger("setStopShooting").onTrue(stopShootInAuto())

val setPriming = EventTrigger("setPriming").onTrue(ShootingState.PRIMING.set())

fun shootOnMoveTestPath(): Command =
    AutoBuilder.followPath(PathPlannerPath.fromPathFile("Test"))

fun depotDoubleCycle(): Command =
    Commands.sequence(
        runPathAndReset("StartToFuelDepotSide"),
        runPath("FuelToDepotStart").alongWith(setShootInAuto()),
        Commands.waitTime(8.sec),
        stopShootInAuto(),
    )

fun depotMainNoShootOnMove(): Command =
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

fun depotMainShootOnMove(): Command =
    Commands.sequence(
        runPathAndReset("StartToFuelDepotSide"),
        runPath("FuelDepotSideToDepotShootOnMove"),
        Commands.waitTime(5.sec).alongWith(IntakingStates.PUMPING.set()),
        IntakingStates.INTAKING.set(),
        Commands.waitTime(0.5.sec),
        runPath("PickupDepot"),
        runPath("ExitDepot"),
        runPath("DepotToFuelShootOnMove")
    )

fun outpostMainShootOnMove(): Command =
    Commands.sequence(
        runPathAndReset(pathName = "StartToFuelDepotSide", mirror = true),
        runPath("FuelOutpostSideToOutpostShootOnMove"),
        Commands.waitTime(5.sec).alongWith(Commands.waitTime(1.0.sec), IntakingStates.PUMPING.set()),
        runPath("OutpostToScatteredFuel")
    )