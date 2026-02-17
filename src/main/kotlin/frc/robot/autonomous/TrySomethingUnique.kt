package frc.robot.subsystems.drive

import choreo.trajectory.SwerveSample
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import frc.robot.lib.getPose2d

val AutoXController = PIDController(0.0, 0.0, 0.0)
val AutoYController = PIDController(0.0, 0.0, 0.0)
val AutoHeadingController = PIDController(7.5, 0.0, 0.0)

fun followTrajectoryBlueSide(sample: SwerveSample) {
    val pose: Pose2d = getPose2d()
    val speeds =
        ChassisSpeeds(
            sample.vx + AutoXController.calculate(pose.getX(), sample.x),
            sample.vy + AutoYController.calculate(pose.getY(), sample.y),
            sample.omega +
                AutoHeadingController.calculate(
                    pose.getRotation().getRadians(),
                    sample.heading
                )
        )
}
