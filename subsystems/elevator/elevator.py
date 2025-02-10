from collections.abc import Callable

from commands2 import Command, Subsystem
from commands2.sysid import SysIdRoutine
from phoenix6 import SignalLogger, StatusSignal, ampere
from phoenix6.configs import TalonFXConfiguration
from phoenix6.configs.config_groups import InvertedValue
from phoenix6.controls import DutyCycleOut, Follower, VoltageOut
from phoenix6.hardware import TalonFX
from wpilib import SmartDashboard
from wpilib.sysid import SysIdRoutineLog
from wpimath import applyDeadband

from constants import Constants
from subsystems.elevator.elevatorconstants import ElevatorConstants


class Elevator(Subsystem):
    def __init__(self) -> None:
        # Left motor is the leader and will be configured below
        self.left = TalonFX(Constants.CanIds.ELEVATOR_LEFT_MOTOR)
        # Right follows left inverted
        self.right = TalonFX(Constants.CanIds.ELEVATOR_RIGHT_MOTOR)
        # Right follows left inverted
        self.right.set_control(Follower(Constants.CanIds.ELEVATOR_LEFT_MOTOR, True))

        cfg = TalonFXConfiguration()
        # TODO check this direction for left motor
        cfg.motor_output.inverted = InvertedValue.CLOCKWISE_POSITIVE
        self.left.configurator.apply(cfg)

        self.dutyCycle = DutyCycleOut(0.0)
        self.voltageOut = VoltageOut(0.0)

        self._sys_id_routine = SysIdRoutine(
            SysIdRoutine.Config(
                # Use default ramp rate (1 V/s) and timeout (10 s)
                # Reduce dynamic voltage to 4 V (TODO may need more)
                stepVoltage=4.0,
                # Log state with SignalLogger class
                recordState=lambda state: SignalLogger.write_string(
                    "SysIdElevator_State", SysIdRoutineLog.stateEnumToString(state)
                ),
            ),
            SysIdRoutine.Mechanism(
                lambda output: self.left.set_control(
                    self.voltageOut.with_output(output)
                ),
                lambda log: None,
                self,
            ),
        )

    def manual_command(self, dutyCycleSupplier: Callable[[], float]) -> Command:
        """
        Constructs a command that can be used to manually operate the elevator.

        :param dutyCycleSupplier: supplies a stream of floats [-1.0, 1.0] where a positive number drives upward.
        :returns: the command
        """
        self.runEnd(
            lambda: self.left.set_control(
                self.dutyCycle.with_output(
                    applyDeadband(
                        dutyCycleSupplier(), ElevatorConstants.ELEVATOR_DEADBAND
                    )
                )
            ),
            lambda: self.left.set_control(self.dutyCycle.with_output(0.0)),
        )

    def sys_id_quasistatic(self, direction: SysIdRoutine.Direction) -> Command:
        return self._sys_id_routine.quasistatic(direction)

    def sys_id_dynamic(self, direction: SysIdRoutine.Direction) -> Command:
        return self._sys_id_routine.dynamic(direction)

    def periodic(self) -> None:
        """
        Overridden to update dashboard.
        """
        current: StatusSignal[ampere] = self.left.get_torque_current()
        SmartDashboard.putNumber("Elevator left amps", current.value())
        current = self.right.get_torque_current()
        SmartDashboard.putNumber("Elevator right amps", current.value())
