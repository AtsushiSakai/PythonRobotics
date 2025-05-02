"""
State Machine

author: Wang Zheng (@Aglargil)

Reference:

- [State Machine]
(https://en.wikipedia.org/wiki/Finite-state_machine)
"""

import string
from urllib.request import urlopen, Request
from base64 import b64encode
from zlib import compress
from io import BytesIO
from collections.abc import Callable
from matplotlib.image import imread
from matplotlib import pyplot as plt


def deflate_and_encode(plantuml_text):
    """
    zlib compress the plantuml text and encode it for the plantuml server.

    Reference https://plantuml.com/en/text-encoding
    """
    plantuml_alphabet = (
        string.digits + string.ascii_uppercase + string.ascii_lowercase + "-_"
    )
    base64_alphabet = (
        string.ascii_uppercase + string.ascii_lowercase + string.digits + "+/"
    )
    b64_to_plantuml = bytes.maketrans(
        base64_alphabet.encode("utf-8"), plantuml_alphabet.encode("utf-8")
    )
    zlibbed_str = compress(plantuml_text.encode("utf-8"))
    compressed_string = zlibbed_str[2:-4]
    return b64encode(compressed_string).translate(b64_to_plantuml).decode("utf-8")


class State:
    def __init__(self, name, on_enter=None, on_exit=None):
        self.name = name
        self.on_enter = on_enter
        self.on_exit = on_exit

    def enter(self):
        print(f"entering <{self.name}>")
        if self.on_enter:
            self.on_enter()

    def exit(self):
        print(f"exiting <{self.name}>")
        if self.on_exit:
            self.on_exit()


class StateMachine:
    def __init__(self, name: str, model=object):
        """Initialize the state machine.

        Args:
            name (str): Name of the state machine.
            model (object, optional): Model object used to automatically look up callback functions
                for states and transitions:
                State callbacks: Automatically searches for 'on_enter_<state>' and 'on_exit_<state>' methods.
                Transition callbacks: When action or guard parameters are strings, looks up corresponding methods in the model.

        Example:
            >>> class MyModel:
            ...     def on_enter_idle(self):
            ...         print("Entering idle state")
            ...     def on_exit_idle(self):
            ...         print("Exiting idle state")
            ...     def can_start(self):
            ...         return True
            ...     def on_start(self):
            ...         print("Starting operation")
            >>> model = MyModel()
            >>> machine = StateMachine("my_machine", model)
        """
        self._name = name
        self._states = {}
        self._events = {}
        self._transition_table = {}
        self._model = model
        self._state: State = None

    def _register_event(self, event: str):
        self._events[event] = event

    def _get_state(self, name):
        return self._states[name]

    def _get_event(self, name):
        return self._events[name]

    def _has_event(self, event: str):
        return event in self._events

    def add_transition(
        self,
        src_state: str | State,
        event: str,
        dst_state: str | State,
        guard: str | Callable = None,
        action: str | Callable = None,
    ) -> None:
        """Add a transition to the state machine.

        Args:
            src_state (str | State): The source state where the transition begins.
                Can be either a state name or a State object.
            event (str): The event that triggers this transition.
            dst_state (str | State): The destination state where the transition ends.
                Can be either a state name or a State object.
            guard (str | Callable, optional): Guard condition for the transition.
                If callable: Function that returns bool.
                If str: Name of a method in the model class.
                If returns True: Transition proceeds.
                If returns False: Transition is skipped.
            action (str | Callable, optional): Action to execute during transition.
                If callable: Function to execute.
                If str: Name of a method in the model class.
                Executed after guard passes and before entering new state.

        Example:
            >>> machine.add_transition(
            ...     src_state="idle",
            ...     event="start",
            ...     dst_state="running",
            ...     guard="can_start",
            ...     action="on_start"
            ... )
        """
        # Convert string parameters to objects if necessary
        self.register_state(src_state)
        self._register_event(event)
        self.register_state(dst_state)

        def get_state_obj(state):
            return state if isinstance(state, State) else self._get_state(state)

        def get_callable(func):
            return func if callable(func) else getattr(self._model, func, None)

        src_state_obj = get_state_obj(src_state)
        dst_state_obj = get_state_obj(dst_state)

        guard_func = get_callable(guard) if guard else None
        action_func = get_callable(action) if action else None
        self._transition_table[(src_state_obj.name, event)] = (
            dst_state_obj,
            guard_func,
            action_func,
        )

    def state_transition(self, src_state: State, event: str):
        if (src_state.name, event) not in self._transition_table:
            raise ValueError(
                f"|{self._name}| invalid transition: <{src_state.name}> : [{event}]"
            )

        dst_state, guard, action = self._transition_table[(src_state.name, event)]

        def call_guard(guard):
            if callable(guard):
                return guard()
            else:
                return True

        def call_action(action):
            if callable(action):
                action()

        if call_guard(guard):
            call_action(action)
            if src_state.name != dst_state.name:
                print(
                    f"|{self._name}| transitioning from <{src_state.name}> to <{dst_state.name}>"
                )
                src_state.exit()
                self._state = dst_state
                dst_state.enter()
        else:
            print(
                f"|{self._name}| skipping transition from <{src_state.name}> to <{dst_state.name}> because guard failed"
            )

    def register_state(self, state: str | State, on_enter=None, on_exit=None):
        """Register a state in the state machine.

        Args:
            state (str | State): The state to register. Can be either a string (state name)
                                or a State object.
            on_enter (Callable, optional): Callback function to be executed when entering the state.
                                         If state is a string and on_enter is None, it will look for
                                         a method named 'on_enter_<state>' in the model.
            on_exit (Callable, optional): Callback function to be executed when exiting the state.
                                        If state is a string and on_exit is None, it will look for
                                        a method named 'on_exit_<state>' in the model.
        Example:
            >>> machine.register_state("idle", on_enter=on_enter_idle, on_exit=on_exit_idle)
            >>> machine.register_state(State("running", on_enter=on_enter_running, on_exit=on_exit_running))
        """
        if isinstance(state, str):
            if on_enter is None:
                on_enter = getattr(self._model, "on_enter_" + state, None)
            if on_exit is None:
                on_exit = getattr(self._model, "on_exit_" + state, None)
            self._states[state] = State(state, on_enter, on_exit)
            return

        self._states[state.name] = state

    def set_current_state(self, state: State | str):
        if isinstance(state, str):
            self._state = self._get_state(state)
        else:
            self._state = state

    def get_current_state(self):
        return self._state

    def process(self, event: str) -> None:
        """Process an event in the state machine.

        Args:
            event: Event name.

        Example:
            >>> machine.process("start")
        """
        if self._state is None:
            raise ValueError("State machine is not initialized")

        if self._has_event(event):
            self.state_transition(self._state, event)
        else:
            raise ValueError(f"Invalid event: {event}")

    def generate_plantuml(self) -> str:
        """Generate PlantUML state diagram representation of the state machine.

        Returns:
            str: PlantUML state diagram code.
        """
        if self._state is None:
            raise ValueError("State machine is not initialized")

        plant_uml = ["@startuml"]
        plant_uml.append("[*] --> " + self._state.name)

        # Generate transitions
        for (src_state, event), (
            dst_state,
            guard,
            action,
        ) in self._transition_table.items():
            transition = f"{src_state} --> {dst_state.name} : {event}"

            # Add guard and action if present
            conditions = []
            if guard:
                guard_name = guard.__name__ if callable(guard) else guard
                conditions.append(f"[{guard_name}]")
            if action:
                action_name = action.__name__ if callable(action) else action
                conditions.append(f"/ {action_name}")

            if conditions:
                transition += "\\n" + " ".join(conditions)

            plant_uml.append(transition)

        plant_uml.append("@enduml")
        plant_uml_text = "\n".join(plant_uml)

        try:
            url = f"http://www.plantuml.com/plantuml/img/{deflate_and_encode(plant_uml_text)}"
            headers = {"User-Agent": "Mozilla/5.0"}
            request = Request(url, headers=headers)

            with urlopen(request) as response:
                content = response.read()

            plt.imshow(imread(BytesIO(content), format="png"))
            plt.axis("off")
            plt.show()
        except Exception as e:
            print(f"Error showing PlantUML: {e}")

        return plant_uml_text
