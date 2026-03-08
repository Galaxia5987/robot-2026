package frc.robot

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.networktables.NetworkTableInstance
import frc.robot.lib.extensions.distanceFromPoint
import frc.robot.lib.extensions.get
import frc.robot.lib.extensions.m
import org.littletonrobotics.junction.Logger
import org.team5987.annotation.LogLevel
import org.team5987.annotation.LoggedOutput

const val areaWeight = 0.6
const val distanceWeight = 0.4

object ObjectDetection {
    private val posesSubscriber =
        NetworkTableInstance.getDefault().getTable("/AdvantageKit/RealsenseVision").getStructArrayTopic(
            "poses",
            Pose3d.struct
        ).subscribe(arrayOf<Pose3d>())

    private val areaSubscriber =
        NetworkTableInstance.getDefault().getTable("/AdvantageKit/RealsenseVision").getFloatArrayTopic(
            "areas"
        ).subscribe(floatArrayOf())


    val optimalCluster: Pose2d?
        get() = findOptimalCluster(
            posesSubscriber.get().map { it.toPose2d() },
            areaSubscriber.get().map { it.toDouble() }
        )

    fun findOptimalCluster(poses: List<Pose2d>, areas: List<Double>): Pose2d? {
        if (poses.isEmpty() || areas.isEmpty()) return null

        val distances = poses.map { p ->
            drive.pose.distanceFromPoint(p.translation)[m]
        }

        val maxDist = distances.maxOrNull() ?: 1.0
        val minDist = distances.minOrNull() ?: 0.0
        val maxArea = areas.maxOrNull() ?: 1.0
        val minArea = areas.minOrNull() ?: 0.0

        var bestIndex = 0
        var highestScore = -1.0
        for (i in poses.indices) {
            val normDist = normalize(distances[i], minDist, maxDist)

            val normArea = normalize(areas[i], minArea, maxArea)

            val score = (areaWeight * normArea) + (distanceWeight * (1.0 - normDist))

            if (score > highestScore) {
                highestScore = score
                bestIndex = i
            }

            Logger.recordOutput("ObjectDetection/Cluster${i}/COM", poses[i])
            Logger.recordOutput("ObjectDetection/Cluster${i}/Score", score)
        }
        Logger.recordOutput("ObjectDetection/optimalCluster", bestIndex)
        Logger.recordOutput("ObjectDetection/optimalClusterScore", highestScore)
        Logger.recordOutput("ObjectDetection/optimalClusterCOM", poses[bestIndex])

        return poses[bestIndex]
    }

    private fun normalize(value: Double, min: Double, max: Double): Double {
        if (max == min) return 0.0 // Avoid division by zero
        return (value - min) / (max - min)
    }

}