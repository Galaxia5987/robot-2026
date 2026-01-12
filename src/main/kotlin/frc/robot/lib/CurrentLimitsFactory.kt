package frc.robot.lib

import com.ctre.phoenix6.configs.CurrentLimitsConfigs
import edu.wpi.first.units.measure.Current
import edu.wpi.first.units.measure.Time
import frc.robot.lib.extensions.amps
import frc.robot.lib.extensions.get
import frc.robot.lib.extensions.sec

fun createCurrentLimits(
    supplyCurrentLimit: Current = 30.amps,
    supplyCurrentPeakDifference: Current = 5.0.amps,
    supplyCurrentLowerTime: Time = 0.4.sec
): CurrentLimitsConfigs =
    CurrentLimitsConfigs().apply {
        SupplyCurrentLimitEnable = true
        StatorCurrentLimitEnable = true

        SupplyCurrentLimit = supplyCurrentLimit[amps]
        StatorCurrentLimit = supplyCurrentLimit[amps] * 2

        SupplyCurrentLowerLimit =
            (supplyCurrentLimit + supplyCurrentPeakDifference)[amps]
        SupplyCurrentLowerTime = supplyCurrentLowerTime[sec]
    }
