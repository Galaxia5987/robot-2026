package frc.robot

import edu.wpi.first.math.geometry.*
import edu.wpi.first.networktables.NetworkTableInstance
import frc.robot.lib.extensions.*
import frc.robot.lib.getTranslation3d
import frc.robot.subsystems.intake.extender.Extender
import org.littletonrobotics.junction.Logger
import kotlin.math.cos
import kotlin.math.sin

const val areaWeight = 0.6
const val distanceWeight = 0.4

object ObjectDetection {
    private val INTAKE_ANGLE = 9.deg

    private val EXTENDER_ANGLE_POSE: Translation3d
        get() =
            getTranslation3d(
                x = Extender.inputs.distance * cos(INTAKE_ANGLE[rad]),
                z = -Extender.inputs.distance * sin(INTAKE_ANGLE[rad])
            )

    private val robotToCamera = {
        Transform3d(
            Translation3d(0.3.m, 0.0.m, 0.5.m) + EXTENDER_ANGLE_POSE,
            Rotation3d(0.0.deg, 0.0.deg, 0.0.deg)
        )
    }

    private val posesSubscriber =
        NetworkTableInstance.getDefault().getTable("/AdvantageKit/RealsenseVision").getStructArrayTopic(
            "poses",
            Pose3d.struct
        ).subscribe(arrayOf<Pose3d>())

    private val areaSubscriber =
        NetworkTableInstance.getDefault().getTable("/AdvantageKit/RealsenseVision").getFloatArrayTopic(
            "areas"
        ).subscribe(floatArrayOf())

    private val poses
        get() = posesSubscriber.get().map {
            Pose3d.kZero.plus(robotToCamera()).plus(Transform3d(Pose3d.kZero, it))
        }


    val optimalCluster: Pose2d?
        get() = findOptimalCluster(
            poses.map { it.toPose2d() },
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