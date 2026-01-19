package frc.robot.subsystems.flywheel

import com.ctre.phoenix6.controls.Follower
import com.ctre.phoenix6.controls.VelocityVoltage
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.lib.extensions.rps
import frc.robot.lib.universal_motor.UniversalTalonFX
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber

object Flywheel : SubsystemBase(), FlywheelVelocitiesCommandFactory {
    private val motor =
        UniversalTalonFX(
            port = MAIN_MOTOR_PORT,
            config = MOTOR_CONFIG,
            subsystem = name,
            simGains = SIM_GAINS
        )

    private val velocityOut = VelocityVoltage(0.0)
    private var setpoint = 0.rps

    init {
        AUXILIARY_MOTORS_PORTS.forEach {
            UniversalTalonFX(
                    port = it.key,
                    config = MOTOR_CONFIG,
                    subsystem = name
                )
                .setControl(Follower(MAIN_MOTOR_PORT, it.value))
        }
    }

    private val calibrationVelocity =
        LoggedNetworkNumber(
            "/Tuning/Flywheel/calibrationFlywheelVelocity",
            40.0
        )

    fun setCalibrationVelocity(): Command = setVelocity {
        calibrationVelocity.get().rps
    }

    fun setVelocity(velocity: () -> AngularVelocity): Command = run {
        setpoint = velocity()
        motor.setControl(velocityOut.withVelocity(setpoint))
    }

    override fun setTarget(value: FlywheelVelocities): Command = runOnce {
        setpoint = value.velocity
        motor.setControl(velocityOut.withVelocity(setpoint))
    }

    override fun periodic() {
        motor.periodic()
        Logger.recordOutput("subsystems/$name/setpoint", setpoint)
    }
}
