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

fun depotBumpDoubleCycle(): Command = Commands.sequence(
    runPath("DepotSideFirstBumpCycle"),
    Commands.waitSeconds(3.0),
    runPath("DepotSideSecondBumpCycle")
)

fun depotOutpostDoubleCycle(): Command = Commands.sequence(
    runPath("DepotSideFirstBumpCycle", mirror = true),
    Commands.waitSeconds(3.0),
    runPath("DepotSideSecondBumpCycle", mirror = true)
)