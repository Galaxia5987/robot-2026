package frc.robot.states

import edu.wpi.first.wpilibj2.command.button.CommandGenericHID
import edu.wpi.first.wpilibj2.command.button.Trigger

enum class ExternalControl(buttonId: Int) {
    AlignmentOverride(0),
    StaticShootingOverride(1),
    ShootOnMoveOverride(2),
    AutoIntakeOverride(3);

    val trigger: Trigger = switchController.button(buttonId)
}

val switchController = CommandGenericHID(1)