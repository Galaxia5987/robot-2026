package frc.robot.states.sensors

import com.ctre.phoenix6.configs.CANrangeConfiguration
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.lib.extensions.cm
import frc.robot.lib.unified_canrange.UnifiedCANRange

val FULL = 1.cm // change the value
val HALF_FULL = 0.5.cm // change the value
val HAS_FUEL = 0.0.cm // change the value


val Spindexer= UnifiedCANRange(1, configuration = CANrangeConfiguration())
val HighLevel1= UnifiedCANRange(2, configuration = CANrangeConfiguration())
val HighLevel2= UnifiedCANRange(3, configuration = CANrangeConfiguration())



val isHalfFull: Trigger = Trigger {
    (HighLevel1.inputs.distance > HALF_FULL).and(HighLevel2.inputs.distance > HALF_FULL
    )
}
val isFull: Trigger = Trigger {
    (HighLevel1.inputs.distance == FULL).and(HighLevel2.inputs.distance == FULL)
}
val hasFuel: Trigger = Trigger { Spindexer.inputs.distance > HAS_FUEL }
