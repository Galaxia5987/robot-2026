package frc.robot.states.sensors

import com.ctre.phoenix6.configs.CANrangeConfiguration
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.lib.extensions.cm
import frc.robot.lib.unified_canrange.UnifiedCANRange

val FULL=1.cm //change the value
val HALF_FULL=0.5.cm //change the value
val HAS_FUEL=0.0.cm //change the value

enum class Sensors(port: Int) {
    Spindexer(1),
    HighLevel1(2),
    HighLevel2(3);

    val canRange = UnifiedCANRange(port, configuration = CANrangeConfiguration())
    val trigger = Trigger(canRange::isInRange)

     val status get() = canRange.inputs.distance


}
val isHalfFull: Trigger= Trigger{
    (Sensors.HighLevel1.status >HALF_FULL).and(Sensors.HighLevel2.status >HALF_FULL)
}

val isFull: Trigger= Trigger{
    (Sensors.HighLevel1.status==FULL).and(Sensors.HighLevel2.status==FULL)
}

val hasFuel: Trigger= Trigger {
    Sensors.Spindexer.status>HAS_FUEL
}