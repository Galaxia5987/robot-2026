package frc.robot.states

import com.ctre.phoenix6.configs.CANrangeConfiguration
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.lib.unified_canrange.UnifiedCANRange

enum class Sensors(port: Int) {
    MidLevelSensor(1),
    Spindexer1(2),
    Spindexer2(3),

    val canRange = UnifiedCANRange(port, configuration = CANrangeConfiguration())
    val trigger = Trigger(canRange::isInRange)
}