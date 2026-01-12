package frc.robot

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation2d
import frc.robot.lib.Mode
import frc.robot.lib.extensions.mm
import frc.robot.lib.extensions.toTransform
import frc.robot.lib.getRotation3d
import frc.robot.lib.getTranslation3d
import frc.robot.sim.RapidReactArena
import frc.robot.subsystems.drive.Drive
import frc.robot.subsystems.drive.ModuleIOs.ModuleIO
import frc.robot.subsystems.drive.ModuleIOs.ModuleIOSim
import frc.robot.subsystems.drive.ModuleIOs.ModuleIOTalonFX
import frc.robot.subsystems.drive.TunerConstants
import frc.robot.subsystems.drive.gyroIOs.GyroIO
import frc.robot.subsystems.drive.gyroIOs.GyroIOPigeon2
import frc.robot.subsystems.drive.gyroIOs.GyroIOSim
import frc.robot.subsystems.shooter.turret.Turret
import frc.robot.subsystems.vision.*
import org.ironmaple.simulation.SimulatedArena
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation

val driveSimulation: SwerveDriveSimulation? =
    if (CURRENT_MODE == Mode.SIM)
        SwerveDriveSimulation(
                Drive.mapleSimConfig,
                Pose2d(3.0, 3.0, Rotation2d())
            )
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
                if (it.key == TURRET_OV_NAME) {
                    VisionIOPhotonVision(
                        it.key,
                        {
                            Pose3d(
                                    it.value.robotToCamera.translation
                                        .rotateAround(
                                            getTranslation3d(z = 441.837.mm),
                                            getRotation3d(
                                                yaw = Turret.inputs.position
                                            )
                                        ),
                                    getRotation3d(
                                        yaw =
                                            it.value.robotToCamera.rotation
                                                .measureZ -
                                                Turret.inputs.position,
                                        pitch =
                                            it.value.robotToCamera.rotation
                                                .measureY
                                    )
                                )
                                .toTransform()
                        },
                        { drive.gyroRotation },
                        { mutableListOf(2, 9, 10, 11) }
                    )
                } else {
                    VisionIOPhotonVision(
                        it.key,
                        { it.value.robotToCamera },
                        { drive.gyroRotation },
                        { mutableListOf(2, 9, 10, 11) }
                    )
                }
            }
        Mode.SIM ->
            OV_NAME_TO_CONFIG.map {
                VisionIOPhotonVisionSim(
                    it.key,
                    { it.value.robotToCamera },
                    { drive.gyroRotation },
                    { mutableListOf(2, 9, 10, 11) },
                    { drive.pose },
                )
            }
        Mode.REPLAY -> emptyList()
    }.toTypedArray()

val vision =
    Vision(
        drive::addGlobalVisionMeasurement,
        drive::addLocalVisionMeasurement,
        *visionIOs
    )
