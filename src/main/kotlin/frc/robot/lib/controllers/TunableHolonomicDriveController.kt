package frc.robot.lib.controllers

import edu.wpi.first.math.controller.HolonomicDriveController
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.trajectory.Trajectory
import edu.wpi.first.units.Units.MetersPerSecond
import edu.wpi.first.units.Units.Radians
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.LinearVelocity
import frc.robot.lib.extensions.log
import frc.robot.subsystems.drive.controller
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber

const val TUNING_PATH = "/Tuning/Alignment"
const val TRANSLATION_TUNING_PATH = "$TUNING_PATH/Translation"
const val ROTATION_TUNING_PATH = "$TUNING_PATH/Rotation"

/**
 * A subclass of [HolonomicDriveController] that allows runtime tuning of PID
 * values using [LoggedNetworkNumber] and logs them to NetworkTables along with
 * the error, position, goal etc...
 *
 * This controller:
 * - Shares the same PID controller for both X and Y translations.
 * - Logs and updates PID gains on every `calculate()` call based on
 * NetworkTables values.
 * - Keeps tuning parameters under `/Tuning/Alignment/Translation` and
 * `/Tuning/Alignment/Rotation`.
 *
 * @param translationController The [PIDController] to use for both X and Y
 * field-relative control.
 * @param thetaController The [ProfiledPIDController] for controlling rotational
 * alignment.
 */
class TunableHolonomicDriveController(
    translationController: PIDController,
    thetaController: ProfiledPIDController,
) :
    HolonomicDriveController(
        translationController,
        translationController,
        thetaController
    ) {
    private val translationKP =
        LoggedNetworkNumber("$TRANSLATION_TUNING_PATH/kP", xController.p)
    private val translationKI =
        LoggedNetworkNumber("$TRANSLATION_TUNING_PATH/kI", xController.i)
    private val translationKD =
        LoggedNetworkNumber("$TRANSLATION_TUNING_PATH/kD", xController.d)

    private val rotationKP =
        LoggedNetworkNumber("$ROTATION_TUNING_PATH/kP", thetaController.p)
    private val rotationKI =
        LoggedNetworkNumber("$ROTATION_TUNING_PATH/kI", thetaController.i)
    private val rotationKD =
        LoggedNetworkNumber("$ROTATION_TUNING_PATH/kD", thetaController.d)

    /**
     * Calculates the next chassis speeds using the current and target poses,
     * while updating PID coefficients from NetworkTables.
     *
     * @param currentPose The current estimated pose of the robot.
     * @param trajectoryPose The desired pose from the trajectory.
     * @param desiredLinearVelocityMetersPerSecond The desired forward velocity.
     * @param desiredHeading The desired final robot heading.
     * @return The chassis speeds to drive the robot.
     */
    override fun calculate(
        currentPose: Pose2d,
        trajectoryPose: Pose2d,
        desiredLinearVelocityMetersPerSecond: Double,
        desiredHeading: Rotation2d,
    ): ChassisSpeeds {
        updatePIDCoefficients()
        controller.log()

        return super.calculate(
            currentPose,
            trajectoryPose,
            desiredLinearVelocityMetersPerSecond,
            desiredHeading
        )
    }

    fun calculate(
        currentPose: Pose2d,
        trajectoryPose: Pose2d,
        desiredLinearVelocity: LinearVelocity,
        desiredHeading: Angle,
    ): ChassisSpeeds =
        calculate(
            currentPose,
            trajectoryPose,
            desiredLinearVelocity.`in`(MetersPerSecond),
            Rotation2d.fromRadians(desiredHeading.`in`(Radians))
        )

    /**
     * Overloaded version of [calculate] that uses a full [Trajectory.State] for
     * input.
     *
     * @param currentPose The current estimated pose of the robot.
     * @param desiredState The desired trajectory state.
     * @param desiredHeading The desired final robot heading.
     * @return The chassis speeds to drive the robot.
     */
    override fun calculate(
        currentPose: Pose2d,
        desiredState: Trajectory.State,
        desiredHeading: Rotation2d,
    ): ChassisSpeeds {
        return calculate(
            currentPose,
            desiredState.poseMeters,
            desiredState.velocityMetersPerSecond,
            desiredHeading
        )
    }

    /**
     * Updates the PID controller coefficients from NetworkTables.
     *
     * This allows real-time tuning of PID constants while the robot is running.
     */
    private fun updatePIDCoefficients() {
        xController.setPID(
            translationKP.get(),
            translationKI.get(),
            translationKD.get()
        )
        yController.setPID(
            translationKP.get(),
            translationKI.get(),
            translationKD.get()
        )
        thetaController.setPID(
            rotationKP.get(),
            rotationKI.get(),
            rotationKD.get()
        )
    }
}
