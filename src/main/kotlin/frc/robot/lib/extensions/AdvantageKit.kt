package frc.robot.lib.extensions

import edu.wpi.first.math.controller.HolonomicDriveController
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.geometry.*
import edu.wpi.first.units.Measure
import edu.wpi.first.units.MutableMeasure
import edu.wpi.first.units.Unit as WPIUnit
import edu.wpi.first.util.struct.Struct
import edu.wpi.first.util.struct.StructSerializable
import edu.wpi.first.wpilibj2.command.button.Trigger
import kotlin.reflect.KProperty
import org.littletonrobotics.junction.AutoLogOutputManager
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.Logger.recordOutput
import org.littletonrobotics.junction.inputs.LoggableInputs
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d

abstract class AutoLogInputs : LoggableInputs {
    fun log(value: Double, key: String? = null) =
        LoggedInput(value, key, LogTable::put, LogTable::get)

    fun log(value: Int, key: String? = null) =
        LoggedInput(value, key, LogTable::put, LogTable::get)

    fun log(value: String, key: String? = null) =
        LoggedInput(value, key, LogTable::put, LogTable::get)

    fun log(value: Boolean, key: String? = null) =
        LoggedInput(value, key, LogTable::put, LogTable::get)

    fun log(value: Long, key: String? = null) =
        LoggedInput(value, key, LogTable::put, LogTable::get)

    fun <T : StructSerializable> log(value: T, key: String? = null) =
        LoggedInput(value, key, LogTable::put, LogTable::get)

    fun <T> log(value: T, struct: Struct<T>, key: String? = null) =
        LoggedInput(
            value,
            key,
            { k, v -> put(k, struct, v) },
            { k, v -> get(k, struct, v) }
        )

    fun <U : WPIUnit> log(value: Measure<U>, key: String? = null) =
        LoggedInput(value, key, LogTable::put, LogTable::get)

    fun <
        U : WPIUnit,
        Base : Measure<WPIUnit>,
        M : MutableMeasure<U, Base, M>,
    > log(value: M, key: String? = null) =
        LoggedInput(value, key, LogTable::put, LogTable::get)

    fun log(value: DoubleArray, key: String? = null) =
        LoggedInput(value, key, LogTable::put, LogTable::get)

    fun log(value: IntArray, key: String? = null) =
        LoggedInput(value, key, LogTable::put, LogTable::get)

    fun log(value: Array<String>, key: String? = null) =
        LoggedInput(value, key, LogTable::put, LogTable::get)

    fun log(value: BooleanArray, key: String? = null) =
        LoggedInput(value, key, LogTable::put, LogTable::get)

    fun log(value: LongArray, key: String? = null) =
        LoggedInput(value, key, LogTable::put, LogTable::get)

    fun <T : StructSerializable> log(value: Array<T>, key: String? = null) =
        LoggedInput(value, key, LogTable::put, LogTable::get)

    private val toLogRunners = mutableListOf<(LogTable) -> Unit>()
    private val fromLogRunners = mutableListOf<(LogTable) -> Unit>()

    inner class LoggedInput<T>(
        private var value: T,
        private val name: String? = null,
        private val toLog: LogTable.(String, T) -> Unit,
        private val fromLog: LogTable.(String, T) -> T,
    ) {
        operator fun getValue(thisRef: Any, property: KProperty<*>) = value
        operator fun setValue(thisRef: Any, property: KProperty<*>, value: T) {
            this.value = value
        }

        operator fun provideDelegate(
            thisRef: Any,
            property: KProperty<*>,
        ): LoggedInput<T> {
            val namespace = this.name ?: property.name
            toLogRunners.add { logTable ->
                this.toLog(logTable, namespace, value)
            }
            fromLogRunners.add { logTable ->
                value = this.fromLog(logTable, namespace, value)
            }
            return this
        }
    }

    override fun fromLog(table: LogTable) {
        fromLogRunners.forEach { it(table) }
    }

    override fun toLog(table: LogTable) {
        toLogRunners.forEach { it(table) }
    }
}

fun enableAutoLogOutputFor(vararg roots: Any) {
    val autoLogOutputClazz = AutoLogOutputManager::class.java
    val objectClazz = Object::class.java
    for (root in roots) {
        val method =
            autoLogOutputClazz.getDeclaredMethod("addObject", objectClazz)
        method.isAccessible = true
        method.invoke(null, root)
    }
}

fun Any.log(prefix: String, key: String) {
    val fullLoggingPath = "$prefix/$key"
    when (this) {
        is String -> recordOutput(fullLoggingPath, this)
        is Int -> recordOutput(fullLoggingPath, this)
        is Double -> recordOutput(fullLoggingPath, this)
        is Boolean -> recordOutput(fullLoggingPath, this)
        is Measure<*> -> recordOutput(fullLoggingPath, this)
        is StructSerializable -> recordOutput(fullLoggingPath, this)
        is LoggedMechanism2d -> recordOutput(fullLoggingPath, this)
        is Trigger -> recordOutput(fullLoggingPath, this)
        else -> recordOutput(fullLoggingPath, this.toString())
    }
}

fun Map<String, Any>.log(loggingPath: String = "") {
    forEach { (key, value) -> value.log(loggingPath, key) }
}

fun PIDController.log(loggingName: String) {
    mapOf(
            "setpoint" to setpoint,
            "error" to error,
            "atSetpoint" to atSetpoint()
        )
        .log(loggingName)
}

fun ProfiledPIDController.log(loggingName: String) {

    mapOf(
            "goal" to goal.position,
            "positionSetpoint" to setpoint.position,
            "error" to positionError,
            "velocitySetpoint" to setpoint.velocity,
            "velocityError" to velocityError,
            "goalVelocity" to goal.velocity,
            "positionTolerance" to positionTolerance,
            "velocityTolerance" to velocityTolerance,
            "maxVelocity" to constraints.maxVelocity,
            "maxAcceleration" to constraints.maxAcceleration,
            "atSetpoint" to atSetpoint(),
            "atGoal" to atGoal()
        )
        .log(loggingName)
}

fun HolonomicDriveController.log() {
    xController.log("XController")
    yController.log("YController")
    thetaController.log("ThetaController")
    recordOutput("Alignment/Controllers/AtGoal", atReference())
}

// ```
// This provides a replacement for the @AutoLog annotation as well as the ability to manually
// register @AutoLogOutput roots.
// Here is an example of using auto-logged inputs in kotlin:
// ```
// class ArmInputs: AutoLogInputs() {
//    var angle by log(Rotation2d())
//    var voltage by log(Volts.mutable(0.0), "ArmVoltage")
//    var statorCurrent by log(Amps.mutable(2.0))
// }
// interface ArmIO {
//    fun updateInputs(inputs: ArmInputs) {}
//    fun setVoltage(volts: Voltage) {}
// }
// class Arm: SubsystemBase() {
//    private val inputs = ArmInputs()
//    private val io: ArmIO = ArmIOImpl()
//    override fun periodic() {
//        io.updateInputs(inputs)
//        Logger.processInputs("Arm", inputs)
//    }
// }
// ```
// And here is an example of registering a singleton for the @AutoLogOutput annotation:
// ```
// object LoggedSingleton {
//    init {
//        enableAutoLogOutputFor(this)
//    }
//    @AutoLogOutput
//    fun getValue() {
//        return 2.0
//    }
// }
