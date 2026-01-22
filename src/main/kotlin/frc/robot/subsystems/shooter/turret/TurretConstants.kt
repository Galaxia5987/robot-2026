package frc.robot.subsystems.shooter.turret

import com.ctre.phoenix6.configs.*
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import com.ctre.phoenix6.signals.SensorDirectionValue
import edu.wpi.first.units.measure.AngularVelocity
import frc.robot.lib.Gains
import frc.robot.lib.createCurrentLimits
import frc.robot.lib.extensions.amps
import frc.robot.lib.extensions.deg
import frc.robot.lib.extensions.get
import frc.robot.lib.extensions.rps
import org.team5987.annotation.command_enum.CommandEnum

const val PORT = 0
const val RATIO = 1.0
const val ENCODER_RATIO = 1.0
val SIM_GAINS = Gains(kP = 0.5, kD = 0.075)
val REAL_GAINS = Gains(kP = 0.5, kD = 0.075)

val TOLERANCE = 2.deg
val SETPOINT_TOLERANCE = 1.deg

val ENCODER_ID = 0
val ABSOLUTE_ENCODER_OFFSET = 0.rps

val SOFTWARE_LIMIT_CONFIG =
    SoftwareLimitSwitchConfigs().apply {
        ForwardSoftLimitEnable = true
        ForwardSoftLimitThreshold = 0.73291
        ReverseSoftLimitEnable = true
        ReverseSoftLimitThreshold = -0.005
    }

val ENCODER_CONFIG =
    CANcoderConfiguration().apply {
        MagnetSensor.SensorDirection =
            SensorDirectionValue.CounterClockwise_Positive
        MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.0
        MagnetSensor.MagnetOffset = ABSOLUTE_ENCODER_OFFSET[rps]
    }

val CONFIG =
    TalonFXConfiguration().apply {
        MotorOutput =
            MotorOutputConfigs().apply {
                NeutralMode = NeutralModeValue.Coast
                Inverted = InvertedValue.Clockwise_Positive
            }
        Slot0 = Slot0Configs().apply { Slot0 = REAL_GAINS.toSlotConfig() }
        Feedback =
            FeedbackConfigs().apply {
                SensorToMechanismRatio = RATIO
                RotorToSensorRatio = ENCODER_RATIO
                FeedbackRemoteSensorID = ENCODER_ID
                FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder
            }
        CurrentLimits = createCurrentLimits(20.amps, 5.amps)
    }

@CommandEnum
enum class ConveyorVelocity(val velocity: AngularVelocity) {
    STOP(0.rps),
    START(10.rps),
    SLOW(4.rps),
    REVERSE(-START.velocity),
    REVERSE_SLOW(-SLOW.velocity)
}
