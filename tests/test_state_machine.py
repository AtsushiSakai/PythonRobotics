import conftest

from MissionPlanning.StateMachine.state_machine import StateMachine


def test_transition():
    sm = StateMachine("state_machine")
    sm.add_transition(src_state="idle", event="start", dst_state="running")
    sm.set_current_state("idle")
    sm.process("start")
    assert sm.get_current_state().name == "running"


def test_guard():
    class Model:
        def can_start(self):
            return False

    sm = StateMachine("state_machine", Model())
    sm.add_transition(
        src_state="idle", event="start", dst_state="running", guard="can_start"
    )
    sm.set_current_state("idle")
    sm.process("start")
    assert sm.get_current_state().name == "idle"


def test_action():
    class Model:
        def on_start(self):
            self.start_called = True

    model = Model()
    sm = StateMachine("state_machine", model)
    sm.add_transition(
        src_state="idle", event="start", dst_state="running", action="on_start"
    )
    sm.set_current_state("idle")
    sm.process("start")
    assert model.start_called


def test_plantuml():
    sm = StateMachine("state_machine")
    sm.add_transition(src_state="idle", event="start", dst_state="running")
    sm.set_current_state("idle")
    assert sm.generate_plantuml()


if __name__ == "__main__":
    conftest.run_this_test(__file__)
