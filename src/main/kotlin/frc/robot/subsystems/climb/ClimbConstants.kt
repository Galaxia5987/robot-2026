package frc.robot.subsystems.climb

import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.signals.InvertedValue
import edu.wpi.first.units.measure.Angle
import frc.robot.lib.Gains
import frc.robot.lib.LoggedNetworkGains
import frc.robot.lib.createCurrentLimits
import frc.robot.lib.extensions.deg
import org.team5987.annotation.command_enum.CommandEnum

const val MAIN_PORT = 0
const val AUX_PORT = 1
const val LOCK_PORT = 2

val REAL_GAINS = LoggedNetworkGains("ClimbRealGains", 1.0, kG = 10.0) //TODO calibrations
val SIM_GAINS = Gains(1.0, kG = 10.0)

const val GEAR_RATION = 1.0

val LOCK = 10.deg
val UNLOCK = 0.deg

val TOLERANCE = 0.5.deg

val MOTOR_CONFIG =
    TalonFXConfiguration().apply {
        Slot0 = REAL_GAINS.toSlotConfig()
        MotionMagic = REAL_GAINS.toMotionMagicConfig()
        CurrentLimits = createCurrentLimits()
        MotorOutput =
            MotorOutputConfigs().apply {
                Inverted = InvertedValue.Clockwise_Positive
            }
    }

@CommandEnum
enum class ClimbLevels(val angle: Angle) {
    GROUND(0.deg),
    LOW(69.deg),
    MID(115.deg),
    HIGH(160.deg),
}
