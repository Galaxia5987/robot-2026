package frc.robot.states

import edu.wpi.first.wpilibj2.command.button.CommandGenericHID
import edu.wpi.first.wpilibj2.command.button.Trigger

enum class DriverOverrides(buttonId: Int) {
    AlignmentOverride(0),
    StaticShootingOverride(1),
    ShootOnMoveOverride(12),
    ShootingCalibrationOverride(8),
    AutoIntakeOverride(3);

    val trigger: Trigger = switchController.button(buttonId)
}

private val switchController = CommandGenericHID(1)
