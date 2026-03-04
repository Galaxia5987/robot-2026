package frc.robot.subsystems.intake.roller

import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.drive
import frc.robot.lib.extensions.rps
import frc.robot.lib.universal_motor.MotorLogConfig
import frc.robot.lib.universal_motor.UniversalTalonFX
import org.littletonrobotics.junction.Logger

object Roller : SubsystemBase(), RollerPositionsCommandFactory {
    private val motor =
        UniversalTalonFX(
            port = PORT,
            config = MOTOR_CONFIG,
            simGains = SIM_GAINS,
            subsystem = name,
            logConfig =
                MotorLogConfig(
                    position = false,
                    statorCurrent = false,
                    current = false,
                    velocity = true,
                    absoluteEncoder = false,
                    voltage = true
                )
        )

    private val velocityTorqueCurrentFOC = VelocityTorqueCurrentFOC(0.0)
    private var setpoint: AngularVelocity = 0.rps

    val inputs
        get() = motor.inputs

    override fun setTarget(value: RollerPositions): Command = runOnce {
        setpoint = value.velocity
        motor.setControl(velocityTorqueCurrentFOC.withVelocity(value.velocity))
    }

    private fun setControl(velocity: AngularVelocity) {
        val feedforward =
            drive.chassisSpeeds.vxMetersPerSecond * ROBOT_VELOCITY_MULTIPLIER
        print("Feedforward: $feedforward")
        setpoint = velocity
        motor.setControl(
            velocityTorqueCurrentFOC
                .withVelocity(velocity)
                .withFeedForward(feedforward)
        )
    }

    fun intake(): Command = defer { run { setControl(INTAKE_BASE_SPEED) } }

    override fun periodic() {
        motor.periodic()
        Logger.recordOutput("Subsystems/Roller/setpoint", setpoint)
    }
}
