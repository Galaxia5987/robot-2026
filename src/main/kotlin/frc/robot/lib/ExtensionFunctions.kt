package frc.robot.lib

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj.util.Color
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.WrapperCommand
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import kotlin.math.*
import org.littletonrobotics.junction.LogTable

fun ChassisSpeeds.getSpeed() = hypot(vxMetersPerSecond, vyMetersPerSecond)

fun List<Any>.toDoubleArray(): DoubleArray {
    return this.map { it as Double }.toTypedArray().toDoubleArray()
}

fun List<Any>.toIntArray(): IntArray {
    return this.map { it as Int }.toTypedArray().toIntArray()
}

fun List<Any>.toBooleanArray(): BooleanArray {
    return this.map { it as Boolean }.toTypedArray().toBooleanArray()
}

fun LogTable.put(key: String, defaultValue: List<Any>) {
    when {
        defaultValue.all { it is Double } ->
            put(key, defaultValue.toDoubleArray())
        defaultValue.all { it is Int } -> put(key, defaultValue.toIntArray())
        defaultValue.all { it is Boolean } ->
            put(key, defaultValue.toBooleanArray())
        else ->
            throw IllegalArgumentException(
                "Unsupported List type: ${defaultValue::class.simpleName}"
            )
    }
}

inline fun <reified T : List<Any>> LogTable.get(
    key: String,
    defaultValue: T
): T {
    val type = defaultValue::class

    val result: List<Any> =
        when {
            defaultValue.all { it is Double } ->
                get(key, defaultValue.toDoubleArray()).toList()
            defaultValue.all { it is Int } ->
                get(key, defaultValue.toIntArray()).toList()
            defaultValue.all { it is Boolean } ->
                get(key, defaultValue.toBooleanArray()).toList()
            else ->
                throw IllegalArgumentException(
                    "Unable to LogTable.get List of type: ${type.simpleName}"
                )
        }
    return if (T::class == MutableList::class) result.toMutableList() as T
    else result as T
}

fun Command.handleInterrupt(command: Command): WrapperCommand =
    handleInterrupt {
        command.schedule()
    }

fun Command.finallyDo(command: Command): WrapperCommand =
    finallyDo(
        Runnable {
            this.cancel()
            if (command.isScheduled) command.cancel()
            command.schedule()
        }
    )

fun CommandXboxController.setRumble(strength: Double) {
    this.hid.setRumble(GenericHID.RumbleType.kBothRumble, strength)
}

fun CommandXboxController.rumbleCommand(): Command {
    return Commands.startEnd({ this.setRumble(1.0) }, { this.setRumble(0.0) })
}

fun Any?.ifNotNull(action: (it: Any) -> Unit) {
    if (this != null) {
        action(this)
    }
}

/**
 * Calculates how similar two colors are, based on HSV distance. Returns a value
 * between 0.0 (completely different) and 1.0 (identical).
 */
fun Color.colorSimilarity(color: Color): Double {

    data class HSVColor(
        val hue: Double,
        val saturation: Double,
        val value: Double
    )

    // --- Convert RGB to HSV ---
    fun rgbToHsv(color: Color): HSVColor {
        val rf = color.red
        val gf = color.green
        val bf = color.blue

        val max = max(rf, max(gf, bf))
        val min = min(rf, min(gf, bf))
        val delta = max - min

        // Compute Hue (angle on the color wheel, normalized to [0,1])
        val hue =
            when {
                delta == 0.0 -> 0.0
                max == rf -> ((gf - bf) / delta) % 6.0
                max == gf -> ((bf - rf) / delta) + 2.0
                else -> ((rf - gf) / delta) + 4.0
            }.let {
                ((it * 60 + 360) % 360) / 360.0
            } // convert degrees to [0..1)

        // Compute Saturation (color intensity)
        val saturation = if (max == 0.0) 0.0 else delta / max

        // Compute Value (brightness)
        val value = max

        return HSVColor(hue, saturation, value)
    }

    // --- Convert both colors ---
    val hsvColor1 = rgbToHsv(this)
    val hsvColor2 = rgbToHsv(color)

    // --- Compute differences ---
    // Hue is circular (red near 0 and 1 are close)
    val hueDiff =
        min(
            abs(hsvColor1.hue - hsvColor2.hue),
            1.0 - abs(hsvColor1.hue - hsvColor2.hue)
        )
    val satDiff = abs(hsvColor1.saturation - hsvColor2.saturation)
    val valDiff = abs(hsvColor1.value - hsvColor2.value)

    // --- Weight each component ---
    val hueWeight = 3.0 // hue is most important
    val satWeight = 1.0
    val valWeight = 0.5

    // Combine into one distance value
    val distance =
        sqrt(
            (hueWeight * hueDiff).pow(2) +
                (satWeight * satDiff).pow(2) +
                (valWeight * valDiff).pow(2)
        )

    // Normalize distance to a similarity between 0 and 1
    val maxDistance =
        sqrt(hueWeight.pow(2) + satWeight.pow(2) + valWeight.pow(2))
    val similarity = 1.0 - (distance / maxDistance)

    return similarity.coerceIn(0.0, 1.0)
}

fun <T : Comparable<T>> T.wrapAround(minimumValue: T, maximumValue: T): T {
    if (minimumValue > maximumValue)
        throw IllegalArgumentException(
            "Cannot coerce value to an empty range: maximum $maximumValue is less than minimum $minimumValue."
        )
    if (this < minimumValue) return maximumValue
    if (this > maximumValue) return minimumValue
    return this
}

fun Rotation2d.convertTo360():
    Rotation2d { // Convert angle from (-180,180) -> (0,360)
    val deg = (this.degrees + 360.0) % 360.0
    return Rotation2d.fromDegrees(deg)
}
