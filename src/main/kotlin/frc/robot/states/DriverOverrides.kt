package frc.robot.states

import edu.wpi.first.wpilibj2.command.button.CommandGenericHID
import edu.wpi.first.wpilibj2.command.button.Trigger

enum class DriverOverrides(buttonId: Int) {
    AlignmentOverride(50),
    StaticShootingOverride(40),
    ShootOnMoveOverride(12),
    ShootingCalibrationOverride(8),
    ShootOnMoveCalibrationOverride(9),
    AutoIntakeOverride(3);

    val trigger: Trigger = switchController.button(buttonId)
}

val switchController = CommandGenericHID(1)

val movingCalibrationIncreaseButton = switchController.button(3)
val movingCalibrationDecreaseButton = switchController.button(4)
val staticCalibrationIncreaseButton = switchController.button(1)
val staticCalibrationDecreaseButton = switchController.button(2)
