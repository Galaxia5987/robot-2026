package frc.robot.subsystems.climb

import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.signals.InvertedValue
import edu.wpi.first.units.measure.Distance
import frc.robot.lib.Gains
import frc.robot.lib.createCurrentLimits
import frc.robot.lib.extensions.cm
import frc.robot.lib.extensions.m
import frc.robot.lib.extensions.mm
import org.team5987.annotation.command_enum.CommandEnum

const val MAIN_PORT = 19

val REAL_GAINS =
    Gains(
        1.0,
    ) // TODO: calibrate
val SIM_GAINS = Gains(1.0, kG = 10.0)

const val GEAR_RATIO = 33.0
val SPROCKET_DIAMETER = 36.4.mm

val TOLERANCE = 0.5.cm

val MOTOR_CONFIG =
    TalonFXConfiguration().apply {
        Slot0 = REAL_GAINS.toSlotConfig()
        CurrentLimits = createCurrentLimits()
        MotorOutput =
            MotorOutputConfigs().apply {
                Inverted = InvertedValue.Clockwise_Positive
            }
    }

@CommandEnum
enum class ClimbLevels(val height: Distance) {
    RETRACTED(0.m),
    OPEN(1.m),
}
