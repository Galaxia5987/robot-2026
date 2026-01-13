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

import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Transform3d
import frc.robot.subsystems.vision.VisionIO.*
import java.util.*
import org.photonvision.PhotonCamera
import org.photonvision.PhotonPoseEstimator
import org.photonvision.targeting.PhotonPipelineResult

/** IO implementation for real PhotonVision hardware. */
open class VisionIOPhotonVision(
    name: String,
    protected val robotToCamera: () -> Transform3d,
    private val botRotation: () -> Rotation2d,
    private val tagIdsToFilter: () -> List<Int>
) : VisionIO {
    protected val camera = PhotonCamera(name)
    private val localPoseEstimator =
        PhotonPoseEstimator(
            APRILTAG_LAYOUT,
            PhotonPoseEstimator.PoseStrategy.PNP_DISTANCE_TRIG_SOLVE,
            robotToCamera()
        )

    override fun updateInputs(inputs: VisionIOInputs) {
        inputs.connected = camera.isConnected
        inputs.name = camera.name

        // Read new camera observations
        val tagIds = mutableSetOf<Short>()
        val poseObservations = mutableListOf<PoseObservation>()

        camera.allUnreadResults.forEach { result ->
            // Update latest target observation
            if (result.hasTargets()) {
                inputs.latestTargetObservation =
                    TargetObservation(
                        Rotation2d.fromDegrees(result.bestTarget.yaw),
                        Rotation2d.fromDegrees(result.bestTarget.pitch)
                    )

                // Local Pose Estimation
                val filteredTargets =
                    result.targets.filter { target ->
                        target.fiducialId in tagIdsToFilter()
                    }

                val filteredResult =
                    PhotonPipelineResult(
                        result.metadata,
                        filteredTargets,
                        Optional.empty()
                    )

                localPoseEstimator.robotToCameraTransform = robotToCamera()
                val estimatedPose = localPoseEstimator.update(filteredResult)
                localPoseEstimator.addHeadingData(
                    result.timestampSeconds,
                    botRotation()
                )

                estimatedPose.ifPresent { estimatedRobotPose ->
                    inputs.localEstimatedPose =
                        PoseObservation(
                            estimatedRobotPose.timestampSeconds,
                            estimatedRobotPose.estimatedPose,
                            estimatedRobotPose.targetsUsed
                                .map { it.poseAmbiguity }
                                .average()
                                .takeIf { !it.isNaN() }
                                ?: 0.0,
                            estimatedRobotPose.targetsUsed.size,
                            estimatedRobotPose.targetsUsed
                                .map { it.bestCameraToTarget.translation.norm }
                                .average()
                                .takeIf { !it.isNaN() }
                                ?: 0.0,
                            PoseObservationType.PHOTONVISION
                        )
                }
            } else
                inputs.latestTargetObservation =
                    TargetObservation(Rotation2d(), Rotation2d())

            // Update PhotonPoseEstimator based on gyro readings
            localPoseEstimator.addHeadingData(
                result.timestampSeconds,
                botRotation()
            )

            // Add pose observation
            if (result.multitagResult.isPresent) { // Multitag result
                val multitagResult = result.multitagResult.get()

                // Calculate robot pose
                val fieldToCamera = multitagResult.estimatedPose.best
                val fieldToRobot = fieldToCamera + robotToCamera().inverse()
                val robotPose =
                    Pose3d(fieldToRobot.translation, fieldToRobot.rotation)

                // Calculate the total tag distance
                val totalTagDistance =
                    result.targets.sumOf {
                        it.bestCameraToTarget.translation.norm
                    }

                // Add tag IDs
                tagIds.addAll(multitagResult.fiducialIDsUsed)

                // Add observation
                poseObservations.add(
                    PoseObservation(
                        result.timestampSeconds,
                        robotPose,
                        multitagResult.estimatedPose.ambiguity,
                        multitagResult.fiducialIDsUsed.size,
                        totalTagDistance / result.targets.size,
                        PoseObservationType.PHOTONVISION
                    )
                )
            } else if (result.targets.isNotEmpty()) { // Single tag result
                val target = result.targets[0]

                // Calculate robot pose
                APRILTAG_LAYOUT.getTagPose(target.fiducialId).ifPresent {
                    tagPose ->
                    val fieldToTarget =
                        Transform3d(tagPose.translation, tagPose.rotation)
                    val cameraToTarget = target.bestCameraToTarget
                    val fieldToCamera = fieldToTarget + cameraToTarget.inverse()
                    val fieldToRobot = fieldToCamera + robotToCamera().inverse()
                    val robotPose =
                        Pose3d(fieldToRobot.translation, fieldToRobot.rotation)

                    // Add tag ID
                    tagIds.add(target.fiducialId.toShort())

                    // Add observation
                    poseObservations.add(
                        PoseObservation(
                            result.timestampSeconds,
                            robotPose,
                            target.poseAmbiguity,
                            1,
                            cameraToTarget.translation.norm,
                            PoseObservationType.PHOTONVISION
                        )
                    )
                }
            }
        }

        // Save pose observations and tag IDs to inputs object
        inputs.poseObservations = poseObservations.toTypedArray()
        inputs.tagIds = tagIds.map { it.toInt() }.toIntArray()
    }
}
