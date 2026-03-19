package frc.robot

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import frc.robot.lib.BetterPoseEstimator
import frc.robot.lib.Mode
import frc.robot.subsystems.drive.Drive
import frc.robot.subsystems.drive.ModuleIOs.ModuleIO
import frc.robot.subsystems.drive.ModuleIOs.ModuleIOSim
import frc.robot.subsystems.drive.ModuleIOs.ModuleIOTalonFX
import frc.robot.subsystems.drive.TunerConstants
import frc.robot.subsystems.drive.gyroIOs.GyroIO
import frc.robot.subsystems.drive.gyroIOs.GyroIOPigeon2
import frc.robot.subsystems.drive.gyroIOs.GyroIOSim
import frc.robot.subsystems.vision.*
import org.ironmaple.simulation.SimulatedArena
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation

val MAPLE_SIM_STARTING_POSE = Pose2d(3.0, 3.0, Rotation2d())

fun resetSimulationField() {
    if (CURRENT_MODE != Mode.SIM) return
    drive.resetOdometry(MAPLE_SIM_STARTING_POSE)
    SimulatedArena.getInstance().resetFieldForAuto()
}

fun getMapleSimPose(): Pose2d? = driveSimulation?.simulatedDriveTrainPose

val driveSimulation: SwerveDriveSimulation? =
    if (CURRENT_MODE == Mode.SIM)
        SwerveDriveSimulation(Drive.mapleSimConfig, MAPLE_SIM_STARTING_POSE)
    else null

private val driveModuleIOs =
    arrayOf(
            TunerConstants.FrontLeft,
            TunerConstants.FrontRight,
            TunerConstants.BackLeft,
            TunerConstants.BackRight
        )
        .mapIndexed { index, module ->
            when (CURRENT_MODE) {
                Mode.REAL -> ModuleIOTalonFX(module)
                Mode.SIM -> ModuleIOSim(driveSimulation!!.modules[index])
                Mode.REPLAY -> object : ModuleIO {}
            }
        }
        .toTypedArray()

private val gyroIO =
    when (CURRENT_MODE) {
        Mode.REAL -> GyroIOPigeon2()
        Mode.SIM ->
            GyroIOSim(
                driveSimulation?.gyroSimulation
                    ?: throw Exception("Gyro simulation is null")
            )
        else -> object : GyroIO {}
    }

val drive =
    Drive(
        gyroIO,
        driveModuleIOs,
        driveSimulation?.let { it::setSimulationWorldPose } ?: { _: Pose2d -> }
    )

private val visionIOs =
    when (CURRENT_MODE) {
        Mode.REAL ->
            OV_NAME_TO_CONFIG.map {
                val config = it.value
                VisionIOPhotonVision(
                    it.key,
                    config.robotToCamera,
                    config.botRotation,
                    config.tagIdsToFilter
                )
            }
        Mode.SIM ->
            OV_NAME_TO_CONFIG.map {
                val config = it.value
                VisionIOPhotonVisionSim(
                    it.key,
                    config.robotToCamera,
                    config.botRotation,
                    config.tagIdsToFilter,
                ) {
                    drive.pose
                }
            }
        Mode.REPLAY -> emptyList()
    }.toTypedArray()

val vision =
    Vision(
        BetterPoseEstimator.getInstance()::addVisionObservation,
        drive::resetOdometry,
        *visionIOs
    )
