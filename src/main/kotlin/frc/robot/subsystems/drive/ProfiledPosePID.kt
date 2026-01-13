package frc.robot.subsystems.drive

import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.lib.LoggedNetworkGains
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber
import org.team5987.annotation.LogLevel
import org.team5987.annotation.LoggedOutput

private const val LOGGING_PREFIX = "AutoAlignment"
private const val TUNING_PATH = "/Tuning/ProfiledPosePID"

@LoggedOutput(LogLevel.DEV) var alignmentGoal: Pose2d = Pose2d()

private val xGains = LoggedNetworkGains("X Gains", kP = 5.0, kD = 0.1)

private val yGains = LoggedNetworkGains("Y Gains", kP = 5.0, kD = 0.1)

private val thetaGains = LoggedNetworkGains("Theta Gains", 4.0, kD = 0.15)
private val linearMaxVelocity =
    LoggedNetworkNumber("$TUNING_PATH/linearMaxVelocity", 40.0)
private val linearMaxAcceleration =
    LoggedNetworkNumber("$TUNING_PATH/linearMaxAcceleration", 60.0)

private var rotationalMaxVelocity =
    LoggedNetworkNumber("$TUNING_PATH/rotationMaxVelocity", 80.0)
private var rotationalMaxAcceleration =
    LoggedNetworkNumber("$TUNING_PATH/rotationMaxAcceleration", 100.0)

private val linearLimits
    get() = Constraints(linearMaxVelocity.get(), linearMaxAcceleration.get())

private val rotationalLimits
    get() =
        Constraints(
            rotationalMaxVelocity.get(),
            rotationalMaxAcceleration.get()
        )

@LoggedOutput(LogLevel.DEV, "X controller", LOGGING_PREFIX)
var xController =
    ProfiledPIDController(
        xGains.kP.get(),
        xGains.kI.get(),
        xGains.kD.get(),
        linearLimits
    )

@LoggedOutput(LogLevel.DEV, "Y controller", LOGGING_PREFIX)
var yController =
    ProfiledPIDController(
        yGains.kP.get(),
        yGains.kI.get(),
        yGains.kD.get(),
        linearLimits
    )

@LoggedOutput(LogLevel.DEV, "Theta controller", LOGGING_PREFIX)
var thetaController =
    ProfiledPIDController(
            thetaGains.kP.get(),
            thetaGains.kI.get(),
            thetaGains.kD.get(),
            rotationalLimits
        )
        .apply { enableContinuousInput(-Math.PI, Math.PI) }

@LoggedOutput(LogLevel.DEV, path = LOGGING_PREFIX)
var atGoal: Trigger =
    Trigger(xController::atGoal)
        .and(yController::atGoal)
        .and(thetaController::atGoal)

fun updateProfiledPIDGains() {
    mapOf(
            xController to Pair(xGains, linearLimits),
            yController to Pair(yGains, linearLimits),
            thetaController to Pair(thetaGains, rotationalLimits)
        )
        .forEach { (controller, pair) ->
            controller.setPID(
                pair.first.kP.get(),
                pair.first.kI.get(),
                pair.first.kD.get()
            )
            println("MAXVELOCITY ${pair.second.maxVelocity}")
            println("MAXACCEL ${pair.second.maxAcceleration}")
            controller.constraints = pair.second
        }
}

fun setGoal(desiredPose: Pose2d) {
    updateProfiledPIDGains()
    alignmentGoal = desiredPose
    xController.setGoal(desiredPose.x)
    yController.setGoal(desiredPose.y)
    thetaController.setGoal(desiredPose.rotation.radians)
}

fun resetProfiledPID(botPose: Pose2d, botSpeeds: ChassisSpeeds) {
    xController.reset(botPose.x, botSpeeds.vxMetersPerSecond)
    yController.reset(botPose.y, botSpeeds.vyMetersPerSecond)
    thetaController.reset(
        botPose.rotation.radians,
        botSpeeds.omegaRadiansPerSecond
    )
}

fun setTolerance(pose2d: Pose2d) {
    xController.setTolerance(pose2d.x)
    yController.setTolerance(pose2d.y)
    thetaController.setTolerance(pose2d.rotation.radians)
}

/**
 * Returns field relative chassis speeds to the selected goal.
 * @botPose the current pose of the robot
 */
fun getSpeedSetpoint(botPose: Pose2d): () -> ChassisSpeeds = {
    ChassisSpeeds(
        xController.calculate(botPose.x),
        yController.calculate(botPose.y),
        thetaController.calculate(botPose.rotation.radians)
    )
}
