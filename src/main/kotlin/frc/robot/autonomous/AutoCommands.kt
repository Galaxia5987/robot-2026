package frc.robot.autonomous

import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.path.PathPlannerPath
import edu.wpi.first.wpilibj2.command.Command

internal fun runPath(name: String): Command {
val path = PathPlannerPath.fromPathFile(name)
val startPose= path.pathPoses[0]
return AutoBuilder.resetOdom(startPose).andThen(AutoBuilder.followPath(path))
}

fun BumpToDeputTest():Command= runPath("BumpToDeput")