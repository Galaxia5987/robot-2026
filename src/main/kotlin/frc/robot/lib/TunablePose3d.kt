package frc.robot.lib

import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Distance
import frc.robot.lib.extensions.*
import frc.robot.lib.extensions.get
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber

class TunableTranslation3d(
    key: String,
    x: Distance = 0.mm,
    y: Distance = 0.mm,
    z: Distance = 0.mm
) {
    private val tunableX = LoggedNetworkNumber("$key/x_mm", x[mm])
    private val tunableY = LoggedNetworkNumber("$key/y_mm", y[mm])
    private val tunableZ = LoggedNetworkNumber("$key/z_mm", z[mm])

    fun get(): Translation3d =
        Translation3d(tunableX.get().mm, tunableY.get().mm, tunableZ.get().mm)
}

class TunableRotation3d(
    key: String,
    roll: Angle = 0.deg,
    pitch: Angle = 0.deg,
    yaw: Angle = 0.deg
) {
    private val tunableRoll = LoggedNetworkNumber("$key/roll_deg", roll[deg])
    private val tunablePitch = LoggedNetworkNumber("$key/pitch_deg", pitch[deg])
    private val tunableYaw = LoggedNetworkNumber("$key/yaw_deg", yaw[deg])

    fun get(): Rotation3d =
        Rotation3d(
            tunableRoll.get().deg,
            tunablePitch.get().deg,
            tunableYaw.get().deg
        )
}

class TunablePose3d(
    key: String,
    translation: Translation3d = Translation3d(),
    rotation: Rotation3d = Rotation3d()
) {
    constructor(
        key: String,
        pose: Pose3d
    ) : this(key, pose.translation, pose.rotation)

    private val tunableTranslation =
        TunableTranslation3d(
            "$key/translation",
            translation.x.meters,
            translation.y.meters,
            translation.z.meters
        )

    private val tunableRotation =
        TunableRotation3d(
            "$key/rotation",
            rotation.measureX,
            rotation.measureY,
            rotation.measureZ
        )

    fun get(): Pose3d = Pose3d(tunableTranslation.get(), tunableRotation.get())
}
