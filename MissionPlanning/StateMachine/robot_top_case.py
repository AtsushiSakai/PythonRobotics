from collections.abc import Callable
from state_machine import StateMachine


class EventBus:
    def __init__(self):
        self.subscribers = {}

    def subscribe(self, event: str, callback: Callable):
        if event not in self.subscribers:
            self.subscribers[event] = []
        self.subscribers[event].append(callback)

    def publish(self, event: str):
        if event in self.subscribers:
            for callback in self.subscribers[event]:
                callback()
        else:
            raise ValueError(f"Invalid event: {event}")


class SlamModel:
    def __init__(self, event_bus: EventBus, mapping_success: bool = False):
        self.event_bus = event_bus
        self.mapping_success = mapping_success

    def on_enter_localization(self):
        self.event_bus.publish("top_localization_ready_event")

    def on_enter_mapping(self):
        self.mapping_success = True

    def is_mapping_success(self):
        return self.mapping_success


class PlanningModel:
    def __init__(self, event_bus: EventBus):
        self.event_bus = event_bus

    def on_exit_working(self):
        self.event_bus.publish("top_stop_working_event")


class TopModel:
    def __init__(self, event_bus: EventBus):
        self.event_bus = event_bus

    def on_enter_pre_working(self):
        self.event_bus.publish("slam_start_localization_event")

    def on_enter_working(self):
        self.event_bus.publish("planning_start_working_event")

    def on_enter_mapping(self):
        self.event_bus.publish("planning_start_remote_control_control_event")
        self.event_bus.publish("slam_start_mapping_event")

    def on_exit_mapping(self):
        self.event_bus.publish("planning_stop_remote_control_control_event")
        self.event_bus.publish("slam_stop_mapping_event")


def main():
    event_bus = EventBus()

    slam_model = SlamModel(event_bus)
    planning_model = PlanningModel(event_bus)
    top_model = TopModel(event_bus)

    slam_machine = StateMachine(slam_model, "slam_machine")
    planning_machine = StateMachine(planning_model, "planning_machine")
    top_machine = StateMachine(top_model, "top_machine")

    # fmt: off
    slam_machine.add_transition("idle", "start_localization_event", "localization", "is_mapping_success")
    slam_machine.add_transition("localization", "stop_localization_event", "idle")
    slam_machine.add_transition("idle", "start_mapping_event", "mapping")
    slam_machine.add_transition("mapping", "stop_mapping_event", "idle")

    planning_machine.add_transition("idle", "start_working_event", "working")
    planning_machine.add_transition("idle", "stop_working_event", "idle")
    planning_machine.add_transition("working", "stop_working_event", "idle")
    planning_machine.add_transition("idle", "start_remote_control_control_event", "remote_control_control")
    planning_machine.add_transition("remote_control_control", "stop_remote_control_control_event", "idle")

    top_machine.add_transition("idle", "start_working_event", "pre_working")
    top_machine.add_transition("pre_working", "localization_ready_event", "working")
    top_machine.add_transition("pre_working", "stop_working_event", "idle")
    top_machine.add_transition("working", "stop_working_event", "idle")
    top_machine.add_transition("idle", "start_mapping_event", "mapping")
    top_machine.add_transition("mapping", "stop_mapping_event", "idle")
    # fmt: on
    event_bus.subscribe(
        "slam_start_localization_event",
        lambda: slam_machine.process("start_localization_event"),
    )
    event_bus.subscribe(
        "slam_start_mapping_event",
        lambda: slam_machine.process("start_mapping_event"),
    )
    event_bus.subscribe(
        "slam_stop_mapping_event",
        lambda: slam_machine.process("stop_mapping_event"),
    )
    event_bus.subscribe(
        "planning_start_working_event",
        lambda: planning_machine.process("start_working_event"),
    )
    event_bus.subscribe(
        "planning_start_remote_control_control_event",
        lambda: planning_machine.process("start_remote_control_control_event"),
    )
    event_bus.subscribe(
        "planning_stop_remote_control_control_event",
        lambda: planning_machine.process("stop_remote_control_control_event"),
    )
    event_bus.subscribe(
        "top_localization_ready_event",
        lambda: top_machine.process("localization_ready_event"),
    )
    event_bus.subscribe(
        "top_mapping_ready_event",
        lambda: top_machine.process("mapping_ready_event"),
    )
    event_bus.subscribe(
        "top_stop_working_event",
        lambda: top_machine.process("stop_working_event"),
    )

    def working_task():
        slam_machine.set_current_state("idle")
        planning_machine.set_current_state("idle")
        top_machine.set_current_state("idle")
        # User sends start working event
        top_machine.process("start_working_event")

        # Planning Model finish the task, and send stop working event
        planning_machine.process("stop_working_event")

        print("top_machine: ", top_machine.get_current_state().name)
        print("planning_machine: ", planning_machine.get_current_state().name)
        print("slam_machine: ", slam_machine.get_current_state().name)

    working_task()

    def mapping_task():
        slam_machine.set_current_state("idle")
        planning_machine.set_current_state("idle")
        top_machine.set_current_state("idle")
        # User sends start mapping event
        top_machine.process("start_mapping_event")
        # User sends stop mapping event
        top_machine.process("stop_mapping_event")

        print("top_machine: ", top_machine.get_current_state().name)
        print("planning_machine: ", planning_machine.get_current_state().name)
        print("slam_machine: ", slam_machine.get_current_state().name)

    mapping_task()


if __name__ == "__main__":
    main()
