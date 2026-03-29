package frc.robot.autonomous

import com.pathplanner.lib.path.PathPlannerPath
import edu.wpi.first.math.geometry.Pose2d

fun parseAutoCommandName(name: String): List<Pose2d> {
    if (!name.startsWith(";")) return emptyList()

    return name
        .substringAfter(";")
        .split(";")
        .filter { it.isNotBlank() }
        .flatMap { entry ->
            val shouldMirror = entry.endsWith("|m")
            val pathName = entry.substringBefore("|m")

            val path = PathPlannerPath.fromPathFile(pathName)
            val finalPath = if (shouldMirror) path.mirrorPath() else path

            finalPath.pathPoses
        }
}