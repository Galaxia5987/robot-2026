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

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Transform3d
import frc.robot.subsystems.vision.VisionIO.*
import java.util.*
import org.photonvision.EstimatedRobotPose
import org.photonvision.PhotonCamera
import org.photonvision.PhotonPoseEstimator

/** IO implementation for real PhotonVision hardware. */
open class VisionIOPhotonVision(
    name: String,
    protected val robotToCamera: () -> Transform3d,
    private val botRotation: () -> Rotation2d,
    private val tagIdsToFilter: () -> List<Int>
) : VisionIO {
    protected val camera = PhotonCamera(name)
    private val poseEstimator =
        PhotonPoseEstimator(APRILTAG_LAYOUT, robotToCamera())

    override fun updateInputs(inputs: VisionIOInputs) {
        inputs.connected = camera.isConnected
        inputs.name = camera.name

        // Read new camera observations
        val tagIds = mutableSetOf<Short>()
        val poseObservations = mutableListOf<PoseObservation>()

        camera.allUnreadResults.forEach { result ->
            // Update latest target observation
            if (result.hasTargets()) {
                poseEstimator.robotToCameraTransform = robotToCamera()

                val estimatedOptionalRobotPose: Optional<EstimatedRobotPose> =
                    if (result.multitagResult.isPresent) {
                        poseEstimator.estimateCoprocMultiTagPose(result)
                    } else {
                        poseEstimator.estimatePnpDistanceTrigSolvePose(result)
                    }

                if (estimatedOptionalRobotPose.isEmpty) {
                    return@forEach
                }

                val estimatedRobotPose = estimatedOptionalRobotPose.get()

                poseEstimator.addHeadingData(
                    result.timestampSeconds,
                    botRotation()
                )

                inputs.estimatedPose =
                    PoseObservation(
                        estimatedRobotPose.timestampSeconds,
                        estimatedRobotPose.estimatedPose,
                        estimatedRobotPose.targetsUsed
                            .map { it.poseAmbiguity }
                            .average(),
                        estimatedRobotPose.targetsUsed.size,
                        estimatedRobotPose.targetsUsed
                            .map { it.bestCameraToTarget.translation.norm }
                            .average()
                    )
            }

            // Update PhotonPoseEstimator based on gyro readings
            poseEstimator.addHeadingData(result.timestampSeconds, botRotation())
        }

        // Save pose observations and tag IDs to inputs object
        inputs.tagIds = tagIds.map { it.toInt() }.toIntArray()
    }
}
