package frc.robot.subsystems.intake.extender

import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.CANcoder
import edu.wpi.first.wpilibj.Alert
import edu.wpi.first.wpilibj.Alert.AlertType
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.lib.extensions.get
import frc.robot.lib.extensions.meters
import frc.robot.lib.extensions.toAngle
import frc.robot.lib.universal_motor.UniversalTalonFX
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d

object Extender : SubsystemBase(), ExtenderPositionsCommandFactory {
    private val motor =
        UniversalTalonFX(
            port = PORT,
            config = CONFIG,
            simGains = SIM_GAINS,
            absoluteEncoderOffset = ABSOLUTE_ENCODER_OFFSET,
            gearRatio = GEAR_RATIO,
            linearSystemWheelDiameter = DIAMETER
        )

    private var setpoint = 0.meters
    val atSetpoint = Trigger {
        setpoint.isNear(motor.inputs.distance, TOLERANCE)
    }

    private var mechanism = LoggedMechanism2d(5.0, 5.0)
    private var root = mechanism.getRoot(name, 2.5, 2.5)
    private val ligament =
        root.append(LoggedMechanismLigament2d("ExtenderLigament", 1.0, 0.0))

    private val positionRequest = PositionTorqueCurrentFOC(0.0)
    private val voltageOut = VoltageOut(0.0)

    private val encoder = CANcoder(ENCODER_ID)

    init {
        encoder.configurator.apply(ENCODER_CONFIG)
    }

    val inputs
        get() = motor.inputs

    override fun setTarget(value: ExtenderPositions): Command = runOnce {
        setpoint = value.distance
        motor.setControl(
            positionRequest.withPosition(
                value.distance.toAngle(DIAMETER, GEAR_RATIO)
            )
        )
    }

    // --- Alerts ---
    private val disconnectedAlert =
        Alert("$name's motor is disconnected", AlertType.kError)

    private val connectedAlert =
        Alert("$name's motor is connected", AlertType.kInfo)

    fun isConnected(condition: Boolean) : Command = runOnce {
        if (condition) {
            disconnectedAlert.set(false)
            connectedAlert.set(true)
        }
        else {
            disconnectedAlert.set(true)
            connectedAlert.set(false)
        }
    }

    val setStatus = Trigger { motor.inputs.connected }
        .onChange(isConnected(motor.inputs.connected))

    override fun periodic() {
        motor.periodic()
        Logger.recordOutput(
            "Subsystems/$name/setpoint",
            setpoint[meters],
            meters
        )
        Logger.recordOutput("Subsystems/$name/mechanism", mechanism)
    }
}
