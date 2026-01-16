// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot

import com.pathplanner.lib.commands.FollowPathCommand
import com.pathplanner.lib.commands.PathfindingCommand
import edu.wpi.first.hal.FRCNetComm.tInstances
import edu.wpi.first.hal.FRCNetComm.tResourceType
import edu.wpi.first.hal.HAL
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.PowerDistribution
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler
import frc.robot.lib.Mode
import frc.robot.lib.extensions.enableAutoLogOutputFor
import frc.robot.lib.logged_output.LoggedOutputManager
import org.ironmaple.simulation.SimulatedArena
import org.littletonrobotics.junction.*
import org.littletonrobotics.junction.networktables.NT4Publisher
import org.littletonrobotics.junction.wpilog.WPILOGReader
import org.littletonrobotics.junction.wpilog.WPILOGWriter

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
object Robot : LoggedRobot() {
    private lateinit var autonomousCommand: Command

    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    init {
        // Report Kotlin language usage
        // https://www.chiefdelphi.com/t/do-you-use-kotlin-make-sure-first-knows/447155?u=dan
        HAL.report(
            tResourceType.kResourceType_Language,
            tInstances.kLanguage_Kotlin
        )
        arrayOf(drive).forEach { AutoLogOutputManager.addObject(it) }

        // Initialize logger
        listOf(
                "Project name" to BuildConstants.MAVEN_NAME,
                "Build date" to BuildConstants.BUILD_DATE,
                "Last commit hash" to BuildConstants.GIT_SHA,
                "Last commit timestamp" to BuildConstants.GIT_DATE,
                "Branch" to BuildConstants.GIT_BRANCH
            )
            .forEach { (key, value) -> Logger.recordMetadata(key, value) }
        @Suppress("KotlinConstantConditions")
        Logger.recordMetadata(
            "Diff status",
            when (BuildConstants.DIRTY) {
                0 -> "All changes committed"
                1 -> "Uncommitted changes"
                else -> "Unknown"
            }
        )

        when (CURRENT_MODE) {
            Mode.REAL -> {
                LoggedPowerDistribution.getInstance(
                    1,
                    PowerDistribution.ModuleType.kRev
                )
                Logger.addDataReceiver(WPILOGWriter())
                Logger.addDataReceiver(NT4Publisher())
            }
            Mode.SIM -> Logger.addDataReceiver(NT4Publisher())
            Mode.REPLAY -> {
                setUseTiming(false)
                val logPath = LogFileUtil.findReplayLog()
                Logger.setReplaySource(WPILOGReader(logPath))
                Logger.addDataReceiver(
                    WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_replay"))
                )
            }
        }
        Logger.start()
        RobotContainer // Initialize robot container.

        enableAutoLogOutputFor(this)

        LoggedOutputManager

        DriverStation.silenceJoystickConnectionWarning(true)
        CommandScheduler.getInstance()
            .schedule(
                FollowPathCommand.warmupCommand(),
                PathfindingCommand.warmupCommand()
            )

        val commandCounts = HashMap<String, Int>()
        val logCommandFunction =
            { command: Command, active: Boolean, verb: String ->
                val name = command.name
                val count =
                    commandCounts.getOrDefault(name, 0) +
                        (if (active) 1 else -1)
                commandCounts[name] = count
                Logger.recordOutput(
                    "Commands/Unique/" +
                        name +
                        "_" +
                        Integer.toHexString(command.hashCode()),
                    active
                )
                Logger.recordOutput("Commands/All/$name", count > 0)
            }
        CommandScheduler.getInstance().onCommandInitialize {
            logCommandFunction(it, true, "initialized")
        }
        CommandScheduler.getInstance().onCommandFinish {
            logCommandFunction(it, false, "finished")
        }
        CommandScheduler.getInstance().onCommandInterrupt { command ->
            logCommandFunction(command, false, "interrupted")
        }
    }

    /**
     * This function is called every robot packet, no matter the mode. Use this
     * for items like diagnostics that you want ran during disabled, autonomous,
     * teleoperated and test.
     *
     * This runs after the mode specific periodic functions, but before
     * LiveWindow and SmartDashboard integrated updating.
     */
    override fun robotPeriodic() {
        CommandScheduler.getInstance().run()
    }

    /**
     * This autonomous (along with the chooser code above) shows how to select
     * between different autonomous modes using the dashboard. The sendable
     * chooser code works with the Java SmartDashboard. If you prefer the
     * LabVIEW Dashboard, remove all of the chooser code and uncomment the
     * getString line to get the auto name from the text box below the Gyro
     *
     * You can add additional auto modes by adding additional comparisons to the
     * switch structure below with additional strings. If using the
     * SendableChooser make sure to add them to the chooser code above as well.
     */
    override fun autonomousInit() {
        // Make sure command is compiled beforehand, otherwise there will be a delay.
        autonomousCommand = RobotContainer.getAutonomousCommand()

        // Schedule the autonomous command
        CommandScheduler.getInstance().schedule(autonomousCommand)
    }

    /** This function is called periodically during autonomous. */
    override fun autonomousPeriodic() {}

    /** This function is called once when teleop is enabled. */
    override fun teleopInit() {
        if (::autonomousCommand.isInitialized) {
            autonomousCommand.cancel()
        }
    }

    override fun simulationPeriodic() {
        val arena = SimulatedArena.getInstance()
        arena.simulationPeriodic()
    }

    /** This function is called periodically during operator control. */
    override fun teleopPeriodic() {}

    /** This function is called once when the robot is disabled. */
    override fun disabledInit() {
        RobotContainer.resetSimulationField()
    }

    /** This function is called periodically when disabled. */
    override fun disabledPeriodic() {}

    /** This function is called once when test mode is enabled. */
    override fun testInit() {
        CommandScheduler.getInstance().cancelAll()
    }

    /** This function is called periodically during test mode. */
    override fun testPeriodic() {}
}
