package frc.robot

import com.pathplanner.lib.auto.AutoBuilder
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller
import edu.wpi.first.wpilibj2.command.button.Trigger
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import frc.robot.field_constants.ALLIANCE_ZONE
import frc.robot.lib.Mode
import frc.robot.lib.extensions.enableAutoLogOutputFor
import frc.robot.lib.extensions.not
import frc.robot.states.setpoints_manager.ShootingType
import frc.robot.states.setpoints_manager.shootingType
import frc.robot.states.shooting.ShootingState
import frc.robot.subsystems.drive.DriveCommands
import frc.robot.subsystems.sensors.Sensors
import frc.robot.subsystems.shooter.flywheel.Flywheel
import frc.robot.subsystems.shooter.hood.Hood
import frc.robot.subsystems.shooter.pre_shooter.PreShooter
import frc.robot.subsystems.shooter.turret.Turret
import frc.robot.subsystems.shooter.turret.Turret.setAngle
import frc.robot.subsystems.shooter.turret.turretAngleToHub
import org.ironmaple.simulation.SimulatedArena
import org.littletonrobotics.junction.AutoLogOutput
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser

object RobotContainer {
    private val driverController = CommandPS5Controller(0)
    private val autoChooser: LoggedDashboardChooser<Command>

    // Shooting state machine triggers
    private object Shooting {
        private val isInTeamZone = Trigger {
            ALLIANCE_ZONE.contains(drive.pose.translation)
        }

        private val dontShoot = driverController.L1()

        private val canShoot =
            Trigger { isOurHubActive }.and(isInTeamZone).and(dontShoot.negate())

        private val atGoal =
            Hood.atSetpoint
                .and(Turret.atSetpoint)
                .and(Flywheel.atSetpoint)
                .and(PreShooter.atSetpoint)

        private val isShootingOnMove = Trigger {
            shootingType == ShootingType.SHOOT_ON_MOVE
        }

        init {
            canShoot.negate().onTrue(ShootingState.IDLE.set())

            ShootingState.IDLE.trigger
                .and(canShoot)
                .onTrue(ShootingState.PRIMING.set())

            ShootingState.PRIMING.trigger
                .onTrue(drive.lock().onlyIf(isShootingOnMove))
                .and(atGoal)
                .onTrue(ShootingState.SHOOTING.set())

            ShootingState.SHOOTING.trigger
                .and(atGoal.negate())
                .onTrue(ShootingState.BACKFEEDING.set())

            ShootingState.BACKFEEDING.trigger
                .and(!Sensors.hasFuel)
                .onTrue(ShootingState.PRIMING.set())

            ShootingState.SHOOTING.trigger
                .and(Sensors.hasFuel.negate())
                .onTrue(ShootingState.IDLE.set())
        }
    }

    init {
        drive // Ensure Drive is initialized
        autoChooser =
            LoggedDashboardChooser(
                "Auto Choices",
                AutoBuilder.buildAutoChooser()
            )

        registerAutoCommands()
        configureButtonBindings()
        configureDefaultCommands()

        if (CURRENT_MODE == Mode.SIM) {
            SimulatedArena.getInstance()
                .addDriveTrainSimulation(driveSimulation)
            SimulatedArena.getInstance().resetFieldForAuto()
        }

        enableAutoLogOutputFor(this)
        Shooting
    }

    @AutoLogOutput(key = "MapleSimPose")
    private fun getMapleSimPose(): Pose2d? =
        driveSimulation?.simulatedDriveTrainPose

    private fun configureDefaultCommands() {
        drive.defaultCommand =
            DriveCommands.joystickDrive(
                { -driverController.leftY },
                { -driverController.leftX },
                { -driverController.rightX * 0.8 }
            )
        Turret.defaultCommand = setAngle { turretAngleToHub }
    }

    private fun configureButtonBindings() {
        //        driverController.square().onTrue(Roller.setTarget(RollerPositions.INTAKE))

        // Intake Bindings
        //        driverController.triangle().onTrue(IntakingStates.INTAKING.set())
        //        driverController
        //            .triangle()
        //            .negate()
        //            .and(canCloseIntake)
        //            .onTrue(IntakingStates.CLOSED.set())
        //        driverController
        //            .triangle()
        //            .negate()
        //            .and(cantCloseIntake)
        //            .onTrue(IntakingStates.OPEN.set())
        //
        //        driverController
        //            .cross()
        //            .onTrue(Spindexer.setTarget(SpindexerVelocity.REVERSE_SLOW))
        //        driverController
        //            .circle()
        //            .onTrue(Spindexer.setTarget(SpindexerVelocity.REVERSE))
    }

    fun getAutonomousCommand(): Command = autoChooser.get()

    private fun registerAutoCommands() {
        // SysIds
        autoChooser.addOption(
            "Drive Wheel Radius Characterization",
            DriveCommands.wheelRadiusCharacterization()
        )
        autoChooser.addOption(
            "Drive Simple FF Characterization",
            DriveCommands.feedforwardCharacterization()
        )
        autoChooser.addOption(
            "Drive SysId (Quasistatic Forward)",
            drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward)
        )
        autoChooser.addOption(
            "Drive SysId (Quasistatic Reverse)",
            drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse)
        )
        autoChooser.addOption(
            "Drive SysId (Dynamic Forward)",
            drive.sysIdDynamic(SysIdRoutine.Direction.kForward)
        )
        autoChooser.addOption(
            "Drive SysId (Dynamic Reverse)",
            drive.sysIdDynamic(SysIdRoutine.Direction.kReverse)
        )

        autoChooser.addOption(
            "swerveFFCharacterization",
            DriveCommands.feedforwardCharacterization()
        )
    }

    fun resetSimulationField() {
        if (CURRENT_MODE != Mode.SIM) return

        SimulatedArena.getInstance().resetFieldForAuto()
    }
}
