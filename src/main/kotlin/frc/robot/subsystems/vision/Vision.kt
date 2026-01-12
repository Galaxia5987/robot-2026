// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.vision

import edu.wpi.first.math.Matrix
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N3
import edu.wpi.first.wpilibj.Alert
import edu.wpi.first.wpilibj.Alert.AlertType
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.subsystems.vision.VisionIO.PoseObservation
import kotlin.math.absoluteValue
import kotlin.math.pow
import org.littletonrobotics.junction.Logger

open class Vision(
    private val globalConsumer: VisionConsumer,
    private val localConsumer: VisionConsumer,
    private vararg val ios: VisionIO
) : SubsystemBase() {

    private val inputs = Array(ios.size) { VisionIOInputsAutoLogged() }
    private val disconnectedAlerts =
        Array(ios.size) { index ->
            Alert("Vision camera $index is disconnected.", AlertType.kWarning)
        }

    /**
     * Returns the X angle to the best target, which can be used for simple
     * servoing with vision.
     *
     * @param cameraIndex The index of the camera to use.
     */
    fun getTargetX(cameraIndex: Int): Rotation2d =
        inputs[cameraIndex].latestTargetObservation.tx

    private fun PoseObservation.isInvalid(): Boolean =
        tagCount == 0 || // Must have at least one tag
        (tagCount == 1 &&
                ambiguity > MAX_AMBIGUITY) || // Cannot be high ambiguity
            pose.z.absoluteValue >
                MAX_Z_ERROR || // Must have realistic Z coordinate
            // Must be within the field boundaries
            !(pose.x in 0.0..APRILTAG_LAYOUT.fieldLength &&
                pose.y in 0.0..APRILTAG_LAYOUT.fieldWidth)

    private fun PoseObservation.isValid(): Boolean = !this.isInvalid()

    private fun PoseObservation.calculateStddev(): Pair<Double, Double> {
        val stdFactor = averageTagDistance.pow(2.0) / tagCount
        val linearStddev = LINEAR_STD_DEV_BASELINE * stdFactor
        val angularStddev = ANGULAR_STD_DEV_BASELINE * stdFactor
        return linearStddev to angularStddev
    }

    override fun periodic() {
        // Update inputs and log
        ios.forEachIndexed { index, visionIO ->
            visionIO.updateInputs(inputs[index])
            Logger.processInputs(LOG_PREFIX + inputs[index].name, inputs[index])
        }

        // Initialize logging collections
        val allTagPoses = mutableListOf<Pose3d>()
        val allRobotPoses = mutableListOf<Pose3d>()
        val allRobotPosesAccepted = mutableListOf<Pose3d>()
        val allRobotPosesRejected = mutableListOf<Pose3d>()

        // Loop over cameras
        inputs.forEachIndexed { cameraIndex, inputs ->
            // Update disconnected alert
            disconnectedAlerts[cameraIndex].set(!inputs.connected)

            // Initialize camera-specific logging collections
            val tagPoses = mutableListOf<Pose3d>()
            val robotPoses = mutableListOf<Pose3d>()
            val robotPosesAccepted = mutableListOf<Pose3d>()
            val robotPosesRejected = mutableListOf<Pose3d>()

            // Add tag poses
            inputs.tagIds.forEach {
                APRILTAG_LAYOUT.getTagPose(it).ifPresent(tagPoses::add)
            }

            // Loop over pose observations
            inputs.poseObservations.forEach {
                robotPoses.add(it.pose)

                if (it.isInvalid()) {
                    robotPosesRejected.add(it.pose)
                }

                // Observation is valid
                robotPosesAccepted.add(it.pose)

                var (linearStdDev, angularStdDev) = it.calculateStddev()

                if (it.type == VisionIO.PoseObservationType.MEGATAG_2) {
                    linearStdDev *= LINEAR_STD_DEV_MEGATAG2_FACTOR
                    angularStdDev *= ANGULAR_STD_DEV_MEGATAG2_FACTOR
                }

                if (cameraIndex < OV_NAME_TO_CONFIG.size) {
                    val stddevFactor =
                        OV_NAME_TO_CONFIG[inputs.name]!!.stddevFactor
                    linearStdDev *= stddevFactor
                    angularStdDev *= stddevFactor
                    Logger.recordOutput(
                        "${LOG_PREFIX}${inputs.name}/linearStdDev",
                        linearStdDev
                    )
                    Logger.recordOutput(
                        "${LOG_PREFIX}${inputs.name}/angularStdDev",
                        angularStdDev
                    )
                }

                // Send vision observation
                globalConsumer.accept(
                    it.pose!!.toPose2d(),
                    it.timestamp,
                    VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev)
                )
            }

            if (inputs.localEstimatedPose.isValid()) {
                val (linearStdDevLocal, angularStdDevLocal) =
                    inputs.localEstimatedPose.calculateStddev()
                localConsumer.accept(
                    inputs.localEstimatedPose.pose.toPose2d(),
                    inputs.localEstimatedPose.timestamp,
                    VecBuilder.fill(
                        linearStdDevLocal,
                        linearStdDevLocal,
                        angularStdDevLocal
                    )
                )
            }

            val loggingCameraPrefix = LOG_PREFIX + inputs.name

            // Log camera data
            Logger.recordOutput(
                "$loggingCameraPrefix/TagPoses",
                *tagPoses.toTypedArray()
            )
            Logger.recordOutput(
                "$loggingCameraPrefix/RobotPoses",
                *robotPoses.toTypedArray()
            )
            Logger.recordOutput(
                ("$loggingCameraPrefix/RobotPosesAccepted"),
                *robotPosesAccepted.toTypedArray()
            )
            Logger.recordOutput(
                ("$loggingCameraPrefix/RobotPosesRejected"),
                *robotPosesRejected.toTypedArray()
            )
            allTagPoses.addAll(tagPoses)
            allRobotPoses.addAll(robotPoses)
            allRobotPosesAccepted.addAll(robotPosesAccepted)
            allRobotPosesRejected.addAll(robotPosesRejected)
        }

        // Log summary data
        val loggingSummaryPrefix = "${LOG_PREFIX}Summary"
        Logger.recordOutput(
            "$loggingSummaryPrefix/TagPoses",
            *allTagPoses.toTypedArray()
        )
        Logger.recordOutput(
            "$loggingSummaryPrefix/RobotPoses",
            *allRobotPoses.toTypedArray()
        )
        Logger.recordOutput(
            "$loggingSummaryPrefix/RobotPosesAccepted",
            *allRobotPosesAccepted.toTypedArray()
        )
        Logger.recordOutput(
            "$loggingSummaryPrefix/RobotPosesRejected",
            *allRobotPosesRejected.toTypedArray()
        )
    }

    fun interface VisionConsumer {
        fun accept(
            visionRobotPoseMeters: Pose2d,
            timestampSeconds: Double,
            visionMeasurementStdDevs: Matrix<N3, N1>
        )
    }
}
