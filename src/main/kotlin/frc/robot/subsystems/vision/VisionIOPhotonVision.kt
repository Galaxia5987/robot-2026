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
import frc.robot.subsystems.vision.VisionIO.PoseObservation
import frc.robot.subsystems.vision.VisionIO.VisionIOInputs
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

        camera.allUnreadResults.forEach { result ->
            // Update latest target observation
            if (result.hasTargets()) {
                poseEstimator.robotToCameraTransform = robotToCamera()

                var observation: PoseObservation? = null

                if (result.multitagResult.isPresent) {
                    val estimatedPose =
                        poseEstimator.estimateCoprocMultiTagPose(result)

                    if (estimatedPose.isEmpty) {
                        return@forEach
                    }

                    val pose = estimatedPose.get()
                    poseEstimator.resetHeadingData(
                        pose.timestampSeconds,
                        pose.estimatedPose.rotation
                    )

                    val estimatedRobotPose = estimatedPose.get()

                    observation =
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
                } else {
                    val target = result.targets[0]
                    val tagPose = APRILTAG_LAYOUT.getTagPose(target.fiducialId)
                    if (tagPose.isPresent) {
                        val fieldToTarget =
                            Transform3d(
                                tagPose.get().translation,
                                tagPose.get().rotation
                            )
                        val cameraToTarget = target.bestCameraToTarget
                        val fieldToCamera =
                            fieldToTarget.plus(cameraToTarget.inverse())
                        val fieldToRobot =
                            fieldToCamera.plus(
                                poseEstimator.robotToCameraTransform.inverse()
                            )
                        val robotPose =
                            Pose3d(
                                fieldToRobot.translation,
                                fieldToRobot.rotation
                            )

                        observation =
                            PoseObservation(
                                result.timestampSeconds,
                                robotPose,
                                target.poseAmbiguity,
                                1,
                                cameraToTarget.translation.norm,
                            )
                    }
                }

                inputs.estimatedPose = observation!!
            } else {
                inputs.estimatedPose = null
            }
        }

        // Save pose observations and tag IDs to inputs object
        inputs.tagIds = tagIds.map { it.toInt() }.toIntArray()
    }
}
