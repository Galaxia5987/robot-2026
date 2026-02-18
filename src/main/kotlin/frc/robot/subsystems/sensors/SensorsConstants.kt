package frc.robot.subsystems.sensors

import com.ctre.phoenix6.configs.CANrangeConfiguration
import com.ctre.phoenix6.configs.FovParamsConfigs
import com.ctre.phoenix6.configs.ProximityParamsConfigs
import frc.robot.lib.extensions.get
import frc.robot.lib.extensions.m
import frc.robot.lib.extensions.sec

const val SPINDEXER_SENSOR_PORT = 0
const val TOP_SENSOR_PORT = 1
const val MIDDLE_SENSOR_PORT = 2

private val HAS_FUEL = 0.19.m

// This should be equal to the time it takes for a ball in the preShooter to exit through the
// shooter
val HAS_FUEL_DEBOUNCE = 0.3.sec

const val MIN_SIGNAL_STRENGTH_FOR_MEASUREMENT = 15015.0

val SPINDEXER_SENSOR_CONFIG =
    CANrangeConfiguration().apply {
        ProximityParams =
            ProximityParamsConfigs().apply {
                ProximityThreshold = HAS_FUEL[m]
                MinSignalStrengthForValidMeasurement =
                    MIN_SIGNAL_STRENGTH_FOR_MEASUREMENT
            }
        FovParams =
            FovParamsConfigs().apply {
                FOVCenterX = 0.0
                FOVCenterY = 0.0
                FOVRangeX = 7.0
                FOVRangeY = 7.0
            }
    }
val TOP_SENSOR_CONFIG = CANrangeConfiguration()
val MIDDLE_SENSOR_CONFIG = CANrangeConfiguration().apply {
    ProximityParams =
        ProximityParamsConfigs().apply {
            ProximityThreshold = 0.61
            MinSignalStrengthForValidMeasurement = 2600.0
        }
    FovParams =
        FovParamsConfigs().apply {
            FOVCenterX = 0.0
            FOVCenterY = 0.0
            FOVRangeX = 27.0
            FOVRangeY = 27.0
        }
}
