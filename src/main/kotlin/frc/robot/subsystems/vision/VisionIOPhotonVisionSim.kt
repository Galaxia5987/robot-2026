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
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Transform3d
import org.photonvision.simulation.PhotonCameraSim
import org.photonvision.simulation.SimCameraProperties
import org.photonvision.simulation.VisionSystemSim

/** IO implementation for physics sim using PhotonVision simulator. */
class VisionIOPhotonVisionSim(
    name: String,
    robotToCamera: () -> Transform3d,
    botRotation: () -> Rotation2d,
    tagIdsToFilter: () -> List<Int>,
    private val poseSupplier: () -> Pose2d
) : VisionIOPhotonVision(name, robotToCamera, botRotation, tagIdsToFilter) {

    private val cameraSim: PhotonCameraSim

    init {
        // Initialize vision sim (singleton)
        if (visionSim == null) {
            visionSim =
                VisionSystemSim("main").apply { addAprilTags(APRILTAG_LAYOUT) }
        }

        // Add sim camera
        val cameraProperties = SimCameraProperties()
        cameraSim = PhotonCameraSim(camera, cameraProperties)
        visionSim!!.addCamera(cameraSim, robotToCamera())
    }

    override fun updateInputs(inputs: VisionIO.VisionIOInputs) {
        visionSim?.update(poseSupplier())
        super.updateInputs(inputs)
    }

    companion object {
        private var visionSim: VisionSystemSim? = null
    }
}
