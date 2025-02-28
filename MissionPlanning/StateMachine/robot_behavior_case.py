"""
A case study of robot behavior using state machine

author: Wang Zheng (@Aglargil)
"""

from state_machine import StateMachine


class Robot:
    def __init__(self):
        self.battery = 100
        self.task_progress = 0

        # Initialize state machine
        self.machine = StateMachine("robot_sm", self)

        # Add state transition rules
        self.machine.add_transition(
            src_state="patrolling",
            event="detect_task",
            dst_state="executing_task",
            guard=None,
            action=None,
        )

        self.machine.add_transition(
            src_state="executing_task",
            event="task_complete",
            dst_state="patrolling",
            guard=None,
            action="reset_task",
        )

        self.machine.add_transition(
            src_state="executing_task",
            event="low_battery",
            dst_state="returning_to_base",
            guard="is_battery_low",
        )

        self.machine.add_transition(
            src_state="returning_to_base",
            event="reach_base",
            dst_state="charging",
            guard=None,
            action=None,
        )

        self.machine.add_transition(
            src_state="charging",
            event="charge_complete",
            dst_state="patrolling",
            guard=None,
            action="battery_full",
        )

        # Set initial state
        self.machine.set_current_state("patrolling")

    def is_battery_low(self):
        """Battery level check condition"""
        return self.battery < 30

    def reset_task(self):
        """Reset task progress"""
        self.task_progress = 0
        print("[Action] Task progress has been reset")

    # Modify state entry callback naming convention (add state_ prefix)
    def on_enter_executing_task(self):
        print("\n------ Start Executing Task ------")
        print(f"Current battery: {self.battery}%")
        while self.machine.get_current_state().name == "executing_task":
            self.task_progress += 10
            self.battery -= 25
            print(
                f"Task progress: {self.task_progress}%, Remaining battery: {self.battery}%"
            )

            if self.task_progress >= 100:
                self.machine.process("task_complete")
                break
            elif self.is_battery_low():
                self.machine.process("low_battery")
                break

    def on_enter_returning_to_base(self):
        print("\nLow battery, returning to charging station...")
        self.machine.process("reach_base")

    def on_enter_charging(self):
        print("\n------ Charging ------")
        self.battery = 100
        print("Charging complete!")
        self.machine.process("charge_complete")


# Keep the test section structure the same, only modify the trigger method
if __name__ == "__main__":
    robot = Robot()
    print(robot.machine.generate_plantuml())

    print(f"Initial state: {robot.machine.get_current_state().name}")
    print("------------")

    # Trigger task detection event
    robot.machine.process("detect_task")

    print("\n------------")
    print(f"Final state: {robot.machine.get_current_state().name}")
