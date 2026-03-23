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
import frc.robot.subsystems.intake.roller.Roller

private fun runPath(path: String, mirror: Boolean = false): Command {
    var path = PathPlannerPath.fromPathFile(path)
    if (mirror) {
        path = path.mirrorPath()
    }
    return AutoBuilder.followPath(path)
}

private fun runPathAndReset(
    pathName: String,
    mirror: Boolean = false
): Command {
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

val setRollerHighCurrentLimit =
    EventTrigger("setRollerHighCurrentLimit")
        .onTrue(Roller.setHighCurrentLimits())
val setRollerNormalCurrentLimit =
    EventTrigger("setRollerNormalCurrentLimit")
        .onTrue(Roller.setNormalCurrentLimits())

fun depotMainShootOnMove(): Command =
    Commands.sequence(
        runPath("DepotSidePickupAndReturn"),
        Commands.waitTime(5.sec).alongWith(IntakingStates.PUMPING.set()),
        runPath("DepotPickupAndReturn")
    )

fun outpostMainShootOnMove(): Command =
    Commands.sequence(
        runPath("StartToFuelDepotSide", mirror = true),
        runPath("FuelOutpostSideToOutpostShootOnMove"),
        Commands.waitTime(5.sec)
            .alongWith(
                Commands.waitTime(1.0.sec),
                IntakingStates.PUMPING.set()
            ),
        runPath("OutpostToScatteredFuel")
    )

fun depotDoubleCycle(): Command =
    Commands.sequence(
        runPath("StartToFuelDepotSide"),
        runPath("FuelToDepotShoot"),
        Commands.waitTime(5.sec),
        runPath("DepotShootToScatteredFuel"),
    )

fun bullshitChallenge(): Command =
    Commands.sequence(
        runPath("StartToFarFuelDepotSide", mirror = true),
        runPath("FarFuelDepotSideToShooting", mirror = true),
        Commands.waitTime(5.sec),
        runPath("DepotSideShootingToCloseFuel", mirror = true),
        runPath("FuelToDepotShoot", mirror = true),
    )