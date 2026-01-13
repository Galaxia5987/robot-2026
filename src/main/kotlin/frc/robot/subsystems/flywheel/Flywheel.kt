package frc.robot.subsystems.flywheel

import com.ctre.phoenix6.controls.Follower
import com.ctre.phoenix6.controls.VelocityVoltage
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.signals.MotorAlignmentValue
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.lib.extensions.rps
import frc.robot.lib.universal_motor.UniversalTalonFX
import org.littletonrobotics.junction.AutoLogOutput
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber

object Flywheel : SubsystemBase() {
    private val Motor =
        UniversalTalonFX(
            port = MOTOR_PORT,
            config = MOTOR_CONFIG,
            subsystem = name
        )
    private val Motor2 =
        UniversalTalonFX(
            port = MOTOT_PORT2,
            config = MOTOR_CONFIG,
            subsystem = name
        )
    private val Motor3 =
        UniversalTalonFX(
            port = MOTOR_PORT3,
            config = MOTOR_CONFIG,
            subsystem = name
        )

    private val velocityTorque = VelocityVoltage(0.0)
    private val voltageOut = VoltageOut(0.0)
    @AutoLogOutput private var setpoint = 0.rps

    val velocity
        get() = Motor.inputs.velocity
    init {
        Motor2.setControl(Follower(MOTOR_PORT, MotorAlignmentValue.Aligned))
        Motor3.setControl(Follower(MOTOR_PORT, MotorAlignmentValue.Aligned))
    }

    private val calibrationVelocity =
        LoggedNetworkNumber("/Tuning/calibrationFlywheelVelocity", 40.0)

    fun setCalibrationAngle(): Command = setVelocity {
        calibrationVelocity.get().rps
    }

    fun setVelocity(velocity: AngularVelocity): Command = runOnce {
        setpoint = velocity
        Motor.setControl(velocityTorque.withVelocity(velocity))
    }

    fun setVelocity(velocity: () -> AngularVelocity): Command = run {
        setpoint = velocity.invoke()
        Motor.setControl(velocityTorque.withVelocity(setpoint))
    }

    fun stop() = setVelocity(0.0.rps)

    fun setVoltage(voltage: Voltage) {
        Motor.setControl(voltageOut.withOutput(voltage))
    }

    override fun periodic() {
        Motor.periodic()
    }
}
