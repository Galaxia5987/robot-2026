package frc.robot.subsystems.intake.extender

import com.ctre.phoenix6.controls.NeutralOut
import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.controls.VoltageOut
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.StartEndCommand
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.lib.createDisableTriggerForCoast
import frc.robot.lib.extensions.*
import frc.robot.lib.universal_motor.UniversalTalonFX
import frc.robot.states.intaking.IntakingStates
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d

object Extender : SubsystemBase() {
    private val motor =
        UniversalTalonFX(
            port = PORT,
            config = CONFIG,
            simGains = SIM_GAINS,
            gearRatio = GEAR_RATIO,
            linearSystemWheelDiameter = DIAMETER
        )

    private val positionRequest = PositionVoltage(0.0)
    private val voltageRequest = VoltageOut(0.0)
    private val neutralOut = NeutralOut()

    private var setpoint = 0.meters

    val atSetpoint = Trigger {
        setpoint.isNear(motor.inputs.distance, EXTENDER_SETPOINT_TOLERANCE)
    }

    var lastStallingDistance = 0.m

    private val isStalling =
        Trigger { motor.inputs.statorCurrent > STATOR_STALL_CURRENT }
            .debounce(STALL_DEBOUNCE[sec])

    private var mechanism = LoggedMechanism2d(5.0, 5.0)

    private var root = mechanism.getRoot(name, 2.5, 2.5)
    private val ligament =
        root.append(LoggedMechanismLigament2d("ExtenderLigament", 1.0, 0.0))

    val shouldStop: Trigger =
        isStalling.and(IntakingStates.INTAKING.trigger.negate())

    val inputs
        get() = motor.inputs

    init {
        motor.reset()
        createDisableTriggerForCoast(motor)
    }

    fun setPosition(value: Distance): Command =
        this.runOnce {
            setpoint = value
            motor.setControl(
                positionRequest.withPosition(
                    value.toAngle(DIAMETER, GEAR_RATIO)
                )
            )
        }

    fun setVoltage(value: Voltage): Command =
        this.runOnce { motor.setControl(voltageRequest.withOutput(value)) }

    private fun stop(): Command = setVoltage(0.volts)

    fun open(): Command = setPosition(ExtenderPositions.OPEN.distance)

    fun close(): Command =
        Commands.sequence(
            setVoltage(CLOSE_VOLTAGE),
            Commands.print("FINISHED SETTING VOLTAGE!!!"),
            Commands.waitSeconds(1.0),
            Commands.print("SHOULD STOP IS TRUE!!!"),
            stop()
        )

    fun openAndReset(): Command =
        StartEndCommand(
            {
                motor.setControl(
                    voltageRequest
                        .withOutput(RESET_VOLTAGE)
                        .withIgnoreSoftwareLimits(true)
                )
            },
            {
                motor.reset(FORWARD_LIMIT)
                motor.setControl(neutralOut)
            }
        )

    override fun periodic() {
        motor.periodic()

        if (isStalling.asBoolean) {
            lastStallingDistance = motor.inputs.distance
        }

        Logger.recordOutput(
            "Subsystems/$name/setpoint",
            setpoint[meters],
            meters
        )
        Logger.recordOutput("Subsystems/$name/mechanism", mechanism)
        Logger.recordOutput("Subsystems/$name/isStalling", isStalling)
        Logger.recordOutput("Subsystems/$name/shouldStop", shouldStop)
        Logger.recordOutput(
            "Subsystems/$name/lastStallDistance",
            lastStallingDistance
        )
    }
}
