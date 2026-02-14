package frc.robot.subsystems.sensors

import com.ctre.phoenix6.configs.CANrangeConfiguration
import com.ctre.phoenix6.configs.FovParamsConfigs
import com.ctre.phoenix6.configs.ProximityParamsConfigs
import frc.robot.lib.extensions.cm
import frc.robot.lib.extensions.get
import frc.robot.lib.extensions.m

val SPINDEXER_SENSOR_PORT = 0
val TOP_SENSOR_PORT = 1
val AUX_TOP_SENSOR = 2

private val HAS_FUEL = 0.19.m

const val MIN_SIGNAL_STRENGTH_FOR_MEASUREMENT = 15015.0

val SPINDEXER_SENSOR_CONFIG =
    CANrangeConfiguration().apply {
        ProximityParams =
            ProximityParamsConfigs().apply {
                ProximityThreshold = HAS_FUEL[m]
                MinSignalStrengthForValidMeasurement = MIN_SIGNAL_STRENGTH_FOR_MEASUREMENT
            }
        FovParams = FovParamsConfigs().apply {
            FOVCenterX = 0.0
            FOVCenterY = 0.0
            FOVRangeX = 7.0
            FOVRangeY = 7.0
        }
    }
val TOP_SENSOR_CONFIG = CANrangeConfiguration()
val AUX_TOP_SENSOR_CONFIG = CANrangeConfiguration()
