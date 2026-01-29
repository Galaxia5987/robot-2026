package frc.robot

import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.units.measure.Angle
import frc.robot.lib.extensions.*
import frc.robot.lib.getRotation3d
import frc.robot.lib.getTranslation3d
import frc.robot.subsystems.drive.Drive
import frc.robot.subsystems.intake.extender.Extender
import frc.robot.subsystems.shooter.hood.Hood
import frc.robot.subsystems.shooter.turret.Turret
import frc.robot.subsystems.spindexer.Spindexer
import org.team5987.annotation.LogLevel
import org.team5987.annotation.LoggedOutput

private val swerveModulePose: Array<Translation2d> =
    Drive.getModuleTranslations()

private val kWheelRadius = 0.0508.m

private fun getSwerveModulePoseTurn(
    moduleX: Double,
    moduleY: Double,
    moduleYaw: Angle
): Pose3d {
    return Pose3d(
        Translation3d(moduleX, moduleY, kWheelRadius[m]),
        getRotation3d(yaw = moduleYaw)
    )
}

private fun getSwerveModulePoseDrive(
    moduleX: Double,
    moduleY: Double,
    moduleYaw: Angle,
    modulePitch: Angle
): Pose3d {

    return Pose3d(
        Translation3d(moduleX, moduleY, kWheelRadius[m]),
        getRotation3d(yaw = moduleYaw, pitch = modulePitch)
    )
}

private fun getAllSwerveModulePoseTurn(): Array<Pose3d> {
    val swervePosesTurn: Array<Pose3d> =
        arrayOf(Pose3d(), Pose3d(), Pose3d(), Pose3d())
    for (i in 0..3) {
        swervePosesTurn[i] =
            getSwerveModulePoseTurn(
                swerveModulePose[i].x,
                swerveModulePose[i].y,
                drive.SwerveTurnAngle[i]
            )
    }
    return swervePosesTurn
}

private fun getAllSwerveModulePoseDrive(): Array<Pose3d> {
    val swervePosesDrive: Array<Pose3d> =
        arrayOf(Pose3d(), Pose3d(), Pose3d(), Pose3d())

    for (i in 0..3) {
        swervePosesDrive[i] =
            getSwerveModulePoseDrive(
                swerveModulePose[i].x,
                swerveModulePose[i].y,
                drive.SwerveTurnAngle[i],
                drive.SwerveDriveAngle[i]
            )
    }
    return swervePosesDrive
}

enum class Poses(
    private val translation3d: () -> Translation3d = { getTranslation3d(0.0) },
    private val rotation3d: () -> Rotation3d = { getRotation3d(0.0) },
) {
    INTAKE_EXTENDER({
        getTranslation3d(80.mm, 10.mm, 248.mm) + Extender.inputs.distance.toX()
    }),
    EXTENDING_HOPPER(
        {
            getTranslation3d(346.mm, 10.mm, 235.mm) +
                Extender.inputs.distance.toX()
        },
    ),
    INTAKE_ROLLER_1({ getTranslation3d(323.mm, 10.mm, 197.mm) }),
    INTAKE_ROLLER_2({ getTranslation3d(260.mm, 10.mm, 197.mm) }),
    TURRET(
        { getTranslation3d((-116).mm, 220.5.mm, 355.mm) },
        { Turret.inputs.position.toYaw() }
    ),
    HOOD(
        TURRET.translation3d,
        { Hood.inputs.position.toPitch() + TURRET.rotation3d() }
    ),
    SHOOTER_MAIN_ROLLER({ getTranslation3d((-48).mm, 220.5.mm, 436.mm) }),
    HOOD_ROLLER({ getTranslation3d((-247).mm, 220.5.mm, 489.mm) }),
    SPINDEXER(
        { getTranslation3d((-26).mm, 43.mm, 36.66200.mm) },
        { getRotation3d(yaw = Spindexer.inputs.position) }
    ),
    PRE_SHOOTER_ROLLER({ getTranslation3d((-63).mm, 90.mm, 211.mm) }),
    PRE_SHOOTER_SECOND_ROLLER({ getTranslation3d((-22).mm, 130.mm, 295.mm) }),
    PRE_SHOOTER_THIRD_ROLLER({ getTranslation3d((-23.5).mm, 315.mm, 295.mm) }),
    CLIMB_GRABBER({ getTranslation3d(0.mm) }),
    CLIMB_WRIST({ getTranslation3d(0.mm) });

    val pose: Pose3d
        get() = Pose3d(translation3d(), rotation3d())
}

private val subsystemPoseArray = Array(14) { Pose3d() }

@LoggedOutput(key = "Visualization/mechanismPoses", level = LogLevel.COMP)
fun mechanismPoses(): Array<Pose3d> {
    //    val swerveModulesPoses = getAllSwerveModulePoseDrive()
    //
    //    swerveModulesPoses.forEachIndexed { i, modulePose ->
    //        subsystemPoseArray[2 * i + 1] = modulePose
    //    }

    subsystemPoseArray[0] = Poses.INTAKE_EXTENDER.pose
    subsystemPoseArray[1] = Poses.EXTENDING_HOPPER.pose
    subsystemPoseArray[2] = Poses.INTAKE_ROLLER_1.pose
    subsystemPoseArray[3] = Poses.INTAKE_ROLLER_2.pose
    subsystemPoseArray[4] = Poses.TURRET.pose
    subsystemPoseArray[5] = Poses.HOOD.pose
    subsystemPoseArray[6] = Poses.SHOOTER_MAIN_ROLLER.pose
    subsystemPoseArray[7] = Poses.HOOD_ROLLER.pose
    subsystemPoseArray[8] = Poses.SPINDEXER.pose
    subsystemPoseArray[9] = Poses.PRE_SHOOTER_ROLLER.pose
    subsystemPoseArray[10] = Poses.PRE_SHOOTER_SECOND_ROLLER.pose
    subsystemPoseArray[11] = Poses.PRE_SHOOTER_THIRD_ROLLER.pose
    subsystemPoseArray[12] = Poses.CLIMB_GRABBER.pose
    subsystemPoseArray[13] = Poses.CLIMB_WRIST.pose

    return subsystemPoseArray
}

// For Tuning !!
private val tuningSubsystemPoses =
    arrayOf(
        TunablePose3d(
            "/Tuning/Visualization/IntakeExtender",
            Poses.INTAKE_EXTENDER.pose
        ),
        TunablePose3d(
            "/Tuning/Visualization/ExtendingHopper",
            Poses.EXTENDING_HOPPER.pose
        ),
        TunablePose3d(
            "/Tuning/Visualization/IntakeRoller1",
            Poses.INTAKE_ROLLER_1.pose
        ),
        TunablePose3d(
            "/Tuning/Visualization/IntakeRoller2",
            Poses.INTAKE_ROLLER_2.pose
        ),
        TunablePose3d("/Tuning/Visualization/Turret", Poses.TURRET.pose),
        TunablePose3d("/Tuning/Visualization/Hood", Poses.HOOD.pose),
        TunablePose3d(
            "/Tuning/Visualization/ShooterMainRoller",
            Poses.SHOOTER_MAIN_ROLLER.pose
        ),
        TunablePose3d(
            "/Tuning/Visualization/HoodRoller",
            Poses.HOOD_ROLLER.pose
        ),
        TunablePose3d("/Tuning/Visualization/Spindexer", Poses.SPINDEXER.pose),
        TunablePose3d(
            "/Tuning/Visualization/PreShooterRoller",
            Poses.PRE_SHOOTER_ROLLER.pose
        ),
        TunablePose3d(
            "/Tuning/Visualization/PreShooterSecondRoller",
            Poses.PRE_SHOOTER_SECOND_ROLLER.pose
        ),
        TunablePose3d(
            "/Tuning/Visualization/PreShooterThirdRoller",
            Poses.PRE_SHOOTER_THIRD_ROLLER.pose
        ),
        TunablePose3d(
            "/Tuning/Visualization/ClimbGrabber",
            Poses.CLIMB_GRABBER.pose
        ),
        TunablePose3d(
            "/Tuning/Visualization/ClimbWrist",
            Poses.CLIMB_WRIST.pose
        ),
    )

@LoggedOutput(key = "Visualization/TuningMechanismPoses", level = LogLevel.COMP)
fun getTuningSubsystemPoses(): Array<Pose3d> =
    tuningSubsystemPoses.map { it.get() }.toTypedArray()
