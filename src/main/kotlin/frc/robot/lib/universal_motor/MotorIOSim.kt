package frc.robot.lib.universal_motor

import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.*
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.MomentOfInertia
import edu.wpi.first.wpilibj.Timer
import frc.robot.lib.Gains
import frc.robot.lib.extensions.get
import frc.robot.lib.extensions.kg2m
import frc.robot.lib.extensions.rot
import frc.robot.lib.extensions.toDistance
import frc.robot.lib.motors.TalonFXSim
import frc.robot.lib.motors.TalonType

/**
 * Simulated implementation of [MotorIO] for use during robot simulation.
 *
 * @param momentOfInertia The moment of inertia of the simulated mechanism.
 * @param config The TalonFX configuration used to build the PID controller.
 * @param gearRatio The gear ratio between motor output and mechanism.
 * @param diameter The wheel/spool diameter for computing linear distance.
 */
class MotorIOSim(
    private val momentOfInertia: MomentOfInertia,
    override val config: TalonFXConfiguration,
    private val simGains: Gains,
    private val gearRatio: Double,
    private val diameter: Distance,
    private val logConfig: MotorLogConfig
) : MotorIO {
    override val inputs = LoggedMotorInputs()
    private val profiledPIDController =
        ProfiledPIDController(
            simGains.kP,
            simGains.kI,
            simGains.kD,
            TrapezoidProfile.Constraints(
                config.MotionMagic.MotionMagicCruiseVelocity,
                config.MotionMagic.MotionMagicAcceleration
            )
        )
    private val controller =
        PIDController(simGains.kP, simGains.kI, simGains.kD).apply {
            if (config.ClosedLoopGeneral.ContinuousWrap) {
                enableContinuousInput(0.0, 1.0)
            }
        }
    private val motor =
        TalonFXSim(1, 1.0, momentOfInertia[kg2m], 1.0, TalonType.KRAKEN_FOC)

    init {
        motor.setController(controller)
        motor.setProfiledController(profiledPIDController)
    }

    override fun setRequest(controlRequest: ControlRequest) {
        when (controlRequest) {
            is VelocityVoltage ->
                controlRequest.FeedForward =
                    controlRequest.Velocity * simGains.kV
            is VelocityTorqueCurrentFOC ->
                controlRequest.FeedForward =
                    controlRequest.Velocity * simGains.kV
            is PositionVoltage ->
                controlRequest.FeedForward =
                    controlRequest.Position * simGains.kV
            is PositionTorqueCurrentFOC ->
                controlRequest.FeedForward =
                    controlRequest.Position * simGains.kV
        }

        motor.setControl(controlRequest)
    }

    override fun updateInputs() {
        motor.update(Timer.getFPGATimestamp())
        if (logConfig.current) inputs.current = motor.appliedCurrent
        if (logConfig.statorCurrent)
            inputs.statorCurrent = motor.appliedCurrent * 2.0

        if (logConfig.voltage) inputs.voltage = motor.appliedVoltage
        if (logConfig.velocity) inputs.velocity = motor.velocity
        if (logConfig.position) {
            inputs.position = motor.position.rot
            inputs.distance = inputs.position.toDistance(diameter, gearRatio)
        }
    }
}
