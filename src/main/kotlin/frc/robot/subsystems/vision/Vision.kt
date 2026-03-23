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

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.wpilibj.Alert
import edu.wpi.first.wpilibj.Alert.AlertType
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.lib.BetterPoseEstimator
import frc.robot.subsystems.vision.VisionIO.PoseObservation
import kotlin.math.absoluteValue
import org.littletonrobotics.junction.Logger

open class Vision(
    private val consumer: (BetterPoseEstimator.VisionObservation) -> Unit,
    private val resetOdometryCallback: (Pose2d) -> Unit,
    private vararg val ios: VisionIO
) : SubsystemBase() {
    private val inputs = Array(ios.size) { VisionIOInputsAutoLogged() }
    private val disconnectedAlerts =
        Array(ios.size) { index ->
            Alert("Vision camera $index is disconnected.", AlertType.kWarning)
        }

    private val invalidPosesBuffer = arrayOfNulls<Pose3d>(ios.size)
    private val emptyPoseArray = emptyArray<Pose3d>()

    private var lastOdometryResetTimeStamp = -1.0

    private fun PoseObservation.isInvalid(): Boolean =
        tagCount == 0 || // Must have at least one tag
        (tagCount == 1 &&
                ambiguity > MAX_AMBIGUITY) || // Cannot be high ambiguity
            pose.z.absoluteValue >
                MAX_Z_ERROR || // Must have realistic Z coordinate
            // Must be within the field boundaries
            !(pose.x in 0.0..APRILTAG_LAYOUT.fieldLength &&
                pose.y in 0.0..APRILTAG_LAYOUT.fieldWidth) ||
            averageTagDistance > MAX_DISTANCE_METERS

    override fun periodic() {
        var invalidPosesCount = 0

        // Standard indexed loop prevents Iterator and List/Pair allocations
        for (i in ios.indices) {
            val visionIO = ios[i]
            val cameraInputs = inputs[i]

            // Update IO
            visionIO.updateInputs(cameraInputs)

            // Update logging
            Logger.processInputs(
                "Subsystems/Vision/${inputs[i].name}",
                cameraInputs
            )

            // Update disconnected alert
            disconnectedAlerts[i].set(!cameraInputs.connected)

            if (!cameraInputs.connected || cameraInputs.estimatedPose == null) {
                continue
            }

            val estimatedPose = cameraInputs.estimatedPose
            if (estimatedPose.isInvalid()) {
                invalidPosesBuffer[invalidPosesCount++] = estimatedPose.pose
                continue
            }

            val tagCountDouble = estimatedPose.tagCount.toDouble()
            val avgDist = estimatedPose.averageTagDistance
            val stdFactor = (avgDist * avgDist) / tagCountDouble

            val linearStdDev =
                (LINEAR_STD_DEV_BASELINE * stdFactor) / tagCountDouble
            val angularStdDev =
                (ANGULAR_STD_DEV_BASELINE * stdFactor) / tagCountDouble

            // Pass raw primitives, avoiding VecBuilder.fill() matrix allocations
            val observation =
                BetterPoseEstimator.VisionObservation(
                    estimatedPose.pose,
                    estimatedPose.timestamp,
                    linearStdDev,
                    linearStdDev,
                    angularStdDev
                )

            if (
                estimatedPose.tagCount > 2 &&
                    estimatedPose.timestamp - lastOdometryResetTimeStamp > 2.5
            ) {
                lastOdometryResetTimeStamp = estimatedPose.timestamp
                resetOdometryCallback.invoke(estimatedPose.pose.toPose2d())
            }
            consumer.invoke(observation)
        }

        if (invalidPosesCount == 0) {
            Logger.recordOutput("InvalidVisionMeasurements", *emptyPoseArray)
        } else {
            val outputArray =
                Array(invalidPosesCount) { invalidPosesBuffer[it]!! }
            Logger.recordOutput("InvalidVisionMeasurements", *outputArray)
        }
    }
}
