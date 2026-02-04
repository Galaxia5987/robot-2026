package frc.robot.sim

import frc.robot.calculateVelocity
import frc.robot.drive
import frc.robot.lib.extensions.cm
import frc.robot.lib.extensions.mps
import frc.robot.lib.getTranslation2d
import frc.robot.subsystems.shooter.hood.Hood
import org.ironmaple.simulation.seasonspecific.crescendo2024.NoteOnFly

class MapleSimShooter {
    private val noteOnFly =
        NoteOnFly(
            drive.pose.translation,
            getTranslation2d(0.0, 0.0),
            drive.chassisSpeeds,
            -drive.pose.rotation,
            0.0.cm,
            calculateVelocity(0.0, 0.0, 0.0).mps,
            Hood.inputs.position
        )


}
