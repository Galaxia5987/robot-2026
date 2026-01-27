package frc.robot.subsystems.sensors

import com.ctre.phoenix6.configs.CANrangeConfiguration
import com.ctre.phoenix6.configs.ProximityParamsConfigs
import frc.robot.lib.extensions.cm
import frc.robot.lib.extensions.get
import frc.robot.lib.extensions.m

val SPINDEXER_SENSOR_PORT = 1
val TOP_SENSOR_PORT = 2
val AUX_TOP_SENSOR = 3

private val HAS_FUEL = 5.cm // TODO: change the value

val SPINDEXER_SENSOR_CONFIG =
    CANrangeConfiguration().apply {
        ProximityParams =
            ProximityParamsConfigs().apply { ProximityThreshold = HAS_FUEL[m] }
    }
val TOP_SENSOR_CONFIG = CANrangeConfiguration()
val AUX_TOP_SENSOR_CONFIG = CANrangeConfiguration()
