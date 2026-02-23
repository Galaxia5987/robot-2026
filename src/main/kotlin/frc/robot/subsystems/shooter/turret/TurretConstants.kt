package frc.robot.subsystems.shooter.turret

import com.ctre.phoenix6.configs.*
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import com.ctre.phoenix6.signals.SensorDirectionValue
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue
import frc.robot.lib.Gains
import frc.robot.lib.createCurrentLimits
import frc.robot.lib.extensions.*
import frc.robot.lib.switchable
import frc.robot.states.setpoints_manager.SetpointsManager.isShootingOnMove

const val PORT = 14
const val RATIO = 55.0
const val ENCODER_RATIO = 1.0
val SIM_GAINS = Gains(kP = 0.5, kD = 0.075)
val REAL_GAINS = Gains(kP = 58.0, kV = 5.0, kS = 0.5)

val SETPOINT_TOLERANCE
    get() = isShootingOnMove.asBoolean.switchable(5.deg, 1.5.deg)

const val ENCODER_ID = 5
val ABSOLUTE_ENCODER_OFFSET = (0.5).rot

val FORWARD_LIMIT = 0.668457.rot
val REVERSE_LIMIT = (-0.121582).rot

val SOFTWARE_LIMIT_CONFIG =
    SoftwareLimitSwitchConfigs().apply {
        ForwardSoftLimitEnable = true
        ReverseSoftLimitEnable = true
        ForwardSoftLimitThreshold = FORWARD_LIMIT[rot]
        ReverseSoftLimitThreshold = REVERSE_LIMIT[rot]
    }

val ENCODER_CONFIG =
    CANcoderConfiguration().apply {
        MagnetSensor.SensorDirection =
            SensorDirectionValue.CounterClockwise_Positive
        MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5
        MagnetSensor.MagnetOffset = ABSOLUTE_ENCODER_OFFSET[rotations]
    }

val CONFIG =
    TalonFXConfiguration().apply {
        MotorOutput =
            MotorOutputConfigs().apply {
                NeutralMode = NeutralModeValue.Brake
                Inverted = InvertedValue.Clockwise_Positive
            }
        Slot0 = REAL_GAINS.toSlotConfig().withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign)
        ClosedLoopGeneral =
            ClosedLoopGeneralConfigs().apply { ContinuousWrap = false }
        SoftwareLimitSwitch = SOFTWARE_LIMIT_CONFIG
        Feedback =
            FeedbackConfigs().apply {
                RotorToSensorRatio = RATIO
                SensorToMechanismRatio = ENCODER_RATIO
                FeedbackRemoteSensorID = ENCODER_ID
                FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder
            }
        CurrentLimits = createCurrentLimits(20.amps, 5.amps)
    }
