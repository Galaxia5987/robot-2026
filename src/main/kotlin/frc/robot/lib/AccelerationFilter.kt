package frc.robot.lib

import edu.wpi.first.math.Matrix
import edu.wpi.first.math.Nat
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.estimator.ExtendedKalmanFilter
import edu.wpi.first.math.geometry.Transform2d
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N2
import edu.wpi.first.math.numbers.N6
import edu.wpi.first.wpilibj.Timer
import frc.robot.LOOP_TIME
import kotlin.math.pow

interface AccelerationFilter {
    val estimatedAccelerationX: Double
    val estimatedAccelerationY: Double
    fun update(
        swerveAx: Double,
        swerveAy: Double,
        rioX: Double,
        rioY: Double,
        pigeonX: Double,
        pigeonY: Double,
        gyroOmega: Double,
        gyroAlpha: Double
    )
}

class AccelerationKalmanFusion(
    val rioToCenter: Transform2d,
    measurementStdDevs: Matrix<N6, N1> =
        VecBuilder.fill(0.5, 0.5, 0.1, 0.1, 0.1, 0.1),
    stateStdDevs: Matrix<N2, N1> = VecBuilder.fill(0.1, 0.1)
) : AccelerationFilter {

    private val ekf: ExtendedKalmanFilter<N2, N2, N6>
    private var lastTime: Double = Timer.getFPGATimestamp()
    private val inputs = Matrix(Nat.N2(), Nat.N1())
    private val measurements = Matrix(Nat.N6(), Nat.N1())
    private val expectedMeasurements = Matrix(Nat.N6(), Nat.N1())

    override val estimatedAccelerationX
        get() = ekf.xhat.get(0, 0)

    override val estimatedAccelerationY
        get() = ekf.xhat.get(1, 0)

    init {

        ekf =
            ExtendedKalmanFilter(
                Nat.N2(),
                Nat.N2(),
                Nat.N6(),
                { x, _ -> x },
                { x, u -> getMeasurements(x, u) },
                stateStdDevs,
                measurementStdDevs,
                LOOP_TIME
            )
    }

    private fun getMeasurements(
        x: Matrix<N2, N1>,
        u: Matrix<N2, N1>
    ): Matrix<N6, N1> {
        val ax = x.get(0, 0)
        val ay = x.get(1, 0)
        val omegaSq = u.get(0, 0).pow(2)
        val alpha = u.get(1, 0)

        expectedMeasurements.set(0, 0, ax) // Swerve X
        expectedMeasurements.set(1, 0, ay) // Swerve Y
        expectedMeasurements.set(
            2,
            0,
            ax - (alpha * rioToCenter.y) - (omegaSq * rioToCenter.x)
        ) // RIO X
        expectedMeasurements.set(
            3,
            0,
            ay + (alpha * rioToCenter.x) - (omegaSq * rioToCenter.y)
        ) // RIO Y
        expectedMeasurements.set(4, 0, ax) // Pigeon X
        expectedMeasurements.set(5, 0, ay) // Pigeon Y

        return expectedMeasurements
    }

    override fun update(
        swerveAx: Double,
        swerveAy: Double,
        rioX: Double,
        rioY: Double,
        pigeonX: Double,
        pigeonY: Double,
        gyroOmega: Double,
        gyroAlpha: Double
    ) {
        val currentTime = Timer.getFPGATimestamp()
        val dt = currentTime - lastTime
        lastTime = currentTime

        // input vector
        inputs.set(0, 0, gyroOmega)
        inputs.set(1, 0, gyroAlpha)

        // measurement vector
        measurements.set(0, 0, swerveAx)
        measurements.set(1, 0, swerveAy)
        measurements.set(2, 0, rioX)
        measurements.set(3, 0, rioY)
        measurements.set(4, 0, pigeonX)
        measurements.set(5, 0, pigeonY)

        ekf.predict(inputs, dt)
        ekf.correct(inputs, measurements)
    }
}
// Alpha
// Higher trusts previous estimate
// Lower faster response but more noise
class AccelerationComplementaryFilter(
    private val rioToCenter: Transform2d,
    private val swerveWeight: Double = 0.2,
    private val rioWeight: Double = 0.2,
    private val pigeonWeight: Double = 0.6,
    private val alpha: Double = 0.7
) : AccelerationFilter {

    override var estimatedAccelerationX: Double = 0.0
        private set

    override var estimatedAccelerationY: Double = 0.0
        private set

    override fun update(
        swerveAx: Double,
        swerveAy: Double,
        rioX: Double,
        rioY: Double,
        pigeonX: Double,
        pigeonY: Double,
        gyroOmega: Double,
        gyroAlpha: Double
    ) {
        val omegaSq = gyroOmega.pow(2)

        val rioCenterX =
            rioX + (gyroAlpha * rioToCenter.y) + (omegaSq * rioToCenter.x)
        val rioCenterY =
            rioY - (gyroAlpha * rioToCenter.x) + (omegaSq * rioToCenter.y)

        val measuredAx =
            (swerveAx * swerveWeight) +
                (rioCenterX * rioWeight) +
                (pigeonX * pigeonWeight)
        val measuredAy =
            (swerveAy * swerveWeight) +
                (rioCenterY * rioWeight) +
                (pigeonY * pigeonWeight)

        estimatedAccelerationX =
            (alpha * estimatedAccelerationX) + ((1.0 - alpha) * measuredAx)
        estimatedAccelerationY =
            (alpha * estimatedAccelerationY) + ((1.0 - alpha) * measuredAy)
    }
}
