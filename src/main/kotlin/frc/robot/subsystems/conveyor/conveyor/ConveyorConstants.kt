package frc.robot.subsystems.conveyor.conveyor

import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.signals.InvertedValue
import edu.wpi.first.units.measure.AngularVelocity
import frc.robot.lib.Gains
import frc.robot.lib.LoggedNetworkGains
import frc.robot.lib.createCurrentLimits
import frc.robot.lib.extensions.rps
import org.team5987.annotation.command_enum.CommandEnum

val GAINS = LoggedNetworkGains("conveyorRealGains", 1.0) // TODO calibration

val SIM_GAINS = Gains(1.0)

const val MAIN_MOTOR_ID = 0

const val AUX_MOTOR_ID = 1

const val GEAR_RATIO = 1.0

val SETPOINT_TOLERANCE = 0.5.rps

val MOTOR_CONFIG =
    TalonFXConfiguration().apply {
        Slot0 = GAINS.toSlotConfig()
        CurrentLimits = createCurrentLimits()
        MotorOutput =
            MotorOutputConfigs().apply {
                Inverted = InvertedValue.Clockwise_Positive
            }
    }

@CommandEnum
enum class ConveyorVelocity(val velocity: AngularVelocity) {
    STOP(0.rps),
    START(10.rps),
    SLOW(4.rps),
    REVERSE(-START.velocity),
    REVERSE_SLOW(-SLOW.velocity)
}
