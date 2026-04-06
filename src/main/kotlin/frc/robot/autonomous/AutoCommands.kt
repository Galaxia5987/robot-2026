package frc.robot.autonomous

import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.events.EventTrigger
import com.pathplanner.lib.path.PathConstraints
import com.pathplanner.lib.path.PathPlannerPath
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import frc.robot.lib.BetterPoseEstimator
import frc.robot.lib.extensions.deg_ps
import frc.robot.lib.extensions.deg_ps_ps
import frc.robot.lib.extensions.flipIfNeeded
import frc.robot.lib.extensions.mirror
import frc.robot.lib.extensions.mps
import frc.robot.lib.extensions.mps_ps
import frc.robot.states.intaking.IntakingStates
import frc.robot.states.shooting.ShootingState
import frc.robot.states.shooting.setShootInAuto
import frc.robot.states.shooting.stopShootInAuto
import frc.robot.subsystems.intake.roller.Roller

private fun runPath(path: String, mirror: Boolean = false, pathfindBefore: Boolean = false, pathfindingConstraints: PathConstraints? = null): Command {
    var path = PathPlannerPath.fromPathFile(path)
    if (mirror) {
        path = path.mirrorPath()
    }

    if (pathfindBefore) {
        return AutoBuilder.pathfindThenFollowPath(path, pathfindingConstraints!!)
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

val setIntaking =
    EventTrigger("setIntaking").onTrue(IntakingStates.INTAKING.set())
val setStopIntaking =
    EventTrigger("setStopIntaking").onTrue(IntakingStates.CLOSED.set())
val setShooting = EventTrigger("setShooting").onTrue(setShootInAuto())
val setStopShooting = EventTrigger("setStopShooting").onTrue(stopShootInAuto())

val setPriming = EventTrigger("setPriming").onTrue(ShootingState.PRIMING.set())

var useOdometryOnlyInAuto = false

private fun setUseOnlyOdometryCommand(): Command = Commands.runOnce({
    useOdometryOnlyInAuto = true
})

val setUseNormalVision = EventTrigger("setUseNormalVision").onTrue(Commands.runOnce({
    useOdometryOnlyInAuto = false
}))

val setRollerHighCurrentLimit =
    EventTrigger("setRollerHighCurrentLimit")
        .onTrue(Roller.setHighCurrentLimits())
val setRollerNormalCurrentLimit =
    EventTrigger("setRollerNormalCurrentLimit")
        .onTrue(Roller.setNormalCurrentLimits())

fun depotBumpDoubleCycle(): Command =
    Commands.sequence(
        runPath("DepotSideFirstBumpCycle"),
        setShootInAuto(),
        Commands.waitSeconds(3.0).alongWith(IntakingStates.PUMPING.set()),
        setUseOnlyOdometryCommand(),
        runPath("DepotSideSecondBumpCycle")
    )

fun outpostBumpDoubleCycle(): Command =
    Commands.sequence(
        runPath("DepotSideFirstBumpCycle", mirror = true),
        setShootInAuto(),
        Commands.waitSeconds(3.0),
        setUseOnlyOdometryCommand(),
        runPath("DepotSideSecondBumpCycle", mirror = true)
    )
