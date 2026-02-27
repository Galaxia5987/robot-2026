package frc.robot.subsystems.vision

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Transform3d
import frc.robot.LimelightHelpers
import frc.robot.lib.extensions.toPose3d
import kotlin.properties.Delegates

open class LimeLightVisionIO(
    name: String,
    protected val robotToCamera: () -> Transform3d,
    private val botRotation: () -> Rotation2d,
    private val tagIdsToFilter: () -> List<Int>
) : VisionIO {
    var lastHeartBeat = 0.0;
    val useMegaTag2 = true //set to false to use MegaTag1
    var currentHeartBeat = 0.0
    lateinit var mt1: LimelightHelpers.PoseEstimate
    lateinit var mt2: LimelightHelpers.PoseEstimate

    override fun updateInputs(inputs: VisionIO.VisionIOInputs) {
        if (!useMegaTag2) {

            mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight")

            if (mt1.tagCount != 0) {
                inputs.estimatedPose = VisionIO.PoseObservation(
                    mt1.timestampSeconds,
                    mt1.pose.toPose3d() + robotToCamera().inverse(),
                    mt1.rawFiducials.sumOf { it.ambiguity } / (mt1.rawFiducials.size),
                    mt1.tagCount,
                    mt1.avgTagDist
                )
            }
        } else {

            mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight")

            LimelightHelpers.SetRobotOrientation(
                "limelight",
                botRotation().degrees,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0
            )

            if (mt2.tagCount != 0) {
                inputs.estimatedPose = VisionIO.PoseObservation(
                    mt2.timestampSeconds,
                    mt2.pose.toPose3d() + robotToCamera().inverse(),
                    mt2.rawFiducials.sumOf { it.ambiguity } / (mt2.rawFiducials.size),
                    mt2.tagCount,
                    mt2.avgTagDist
                )

                inputs.connected = mt2.latency > 100
                inputs.tagIds = mt2.rawFiducials.map { it.id }.toIntArray()
            }
        }
        currentHeartBeat = LimelightHelpers.getHeartbeat("limelight")
        inputs.connected = lastHeartBeat < currentHeartBeat
        lastHeartBeat = currentHeartBeat
    }
}
