package frc.robot.states.setpoints_manager.setpoints_state

import edu.wpi.first.units.Measure
import edu.wpi.first.units.Unit
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.lib.extensions.deg
import frc.robot.lib.extensions.rps
import frc.robot.subsystems.shooter.flywheel.Flywheel
import frc.robot.subsystems.shooter.hood.Hood
import frc.robot.subsystems.shooter.pre_shooter.PreShooter
import frc.robot.subsystems.shooter.turret.Turret

enum class FieldLocation(val flywheel: AngularVelocity, val hood: Angle, val turret: Angle) {
    HUB(flywheel = 0.rps, hood = 0.deg, turret = 0.deg),
    CLIMB(flywheel = 0.rps, hood = 0.deg, turret = 0.deg),
    UPPER_BUMPER(flywheel = 0.rps, hood = 0.deg, turret = 0.deg),
    LOWER_BUMPER(flywheel = 0.rps, hood = 0.deg, turret = 0.deg),
    UPPER_TRENCH(flywheel = 0.rps, hood = 0.deg, turret = 0.deg),
    LOWER_TRENCH(flywheel = 0.rps, hood = 0.deg, turret = 0.deg),
    UPPER_CORNER(flywheel = 0.rps, hood = 0.deg, turret = 0.deg),
    LOWER_CORNER(flywheel = 0.rps, hood = 0.deg, turret = 0.deg)
}

var currentArea: FieldLocation = FieldLocation.HUB

val staticShootingMap: Map<SubsystemBase, () -> Measure<out Unit>> = mapOf(
    Turret to { currentArea.turret },
    Hood to { currentArea.hood },
    Flywheel to { currentArea.flywheel },
    PreShooter to { currentArea.flywheel },
)