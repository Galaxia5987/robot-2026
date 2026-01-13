package frc.robot.lib.universal_motor

import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.ControlRequest
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.units.Units.Rotations
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
    private val diameter: Distance
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
        PIDController(simGains.kP, simGains.kI, simGains.kD)
    private val motor =
        TalonFXSim(1, 1.0, momentOfInertia[kg2m], 1.0, TalonType.KRAKEN_FOC)

    init {
        motor.setController(controller)
        motor.setProfiledController(profiledPIDController)
    }

    override fun setRequest(controlRequest: ControlRequest) {
        motor.setControl(controlRequest)
    }

    override fun updateInputs() {
        motor.update(Timer.getTimestamp())
        inputs.current = motor.appliedCurrent
        inputs.position = motor.position.rot
        inputs.voltage = motor.appliedVoltage
        inputs.velocity = motor.velocity
        inputs.distance =
            Rotations.of(motor.position).toDistance(diameter, gearRatio)
    }
}
