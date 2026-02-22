package frc.robot.subsystems.shooter.hood

import com.ctre.phoenix6.configs.*
import com.ctre.phoenix6.signals.*
import edu.wpi.first.units.measure.Angle
import frc.robot.lib.Gains
import frc.robot.lib.createCurrentLimits
import frc.robot.lib.extensions.*
import frc.robot.lib.switchable
import frc.robot.states.setpoints_manager.SetpointsManager.isShootingOnMove
import org.team5987.annotation.command_enum.CommandEnum

const val PORT = 15

const val ENCODER_ID = 4
const val ENCODER_GEAR_RATIO = 2.0

val TOLERANCE
    get() = isShootingOnMove.asBoolean.switchable(4.deg, 0.5.deg)

val SIM_GAINS = Gains(kP = 1.7, kD = 0.3)
val REAL_GAINS = Gains(kP = 270.0, kS = 0.1)

const val GEAR_RATIO = 46.77199935913086

// The actual angle, (relative to the ground) of the hood on the robot.
val HOOD_STARTING_ANGLE = 15.deg

val ABSOLUTE_ENCODER_OFFSET = (-0.005127).rot

val FORWARD_LIMIT = 0.061.rot
val REVERSE_LIMIT = 0.rot

val CROUCH_TOLERANCE = 0.4.m

val ENCODER_CONFIG =
    CANcoderConfiguration().apply {
        MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive
        MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.9
        MagnetSensor.MagnetOffset = -ABSOLUTE_ENCODER_OFFSET[rot]
    }

val CONFIG =
    TalonFXConfiguration().apply {
        MotorOutput =
            MotorOutputConfigs().apply {
                NeutralMode = NeutralModeValue.Brake
                Inverted = InvertedValue.Clockwise_Positive
            }

        SoftwareLimitSwitch =
            SoftwareLimitSwitchConfigs().apply {
                ForwardSoftLimitEnable = true
                ReverseSoftLimitEnable = true
                ForwardSoftLimitThreshold = FORWARD_LIMIT[rot]
                ReverseSoftLimitThreshold = REVERSE_LIMIT[rot]
            }

        Slot0 =
            REAL_GAINS.toSlotConfig()
                .withStaticFeedforwardSign(
                    StaticFeedforwardSignValue.UseClosedLoopSign
                )
        Feedback =
            FeedbackConfigs().apply {
                RotorToSensorRatio = GEAR_RATIO
                SensorToMechanismRatio = ENCODER_GEAR_RATIO
                FeedbackRemoteSensorID = ENCODER_ID
                FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder
            }

        CurrentLimits = createCurrentLimits(15.amps, 5.amps)
    }

@CommandEnum
enum class HoodPositions(var angle: Angle) {
    UP(10.deg),
    DOWN(0.deg)
}
// TODO: real values
