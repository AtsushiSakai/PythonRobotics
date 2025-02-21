from collections.abc import Callable


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


class StateMachine(State):
    def __init__(self, model: object, name: str, on_enter=None, on_exit=None):
        State.__init__(self, name, on_enter, on_exit)
        self.states = {}
        self.events = {}
        self.transition_table = {}
        self._model = model
        self._state: StateMachine = None

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
            src_state: Source state name or State object
            event: Event name or Event object
            dst_state: Destination state name or State object
            guard: Guard function name or callable
            action: Action function name or callable
        """
        # Convert string parameters to objects if necessary
        self.register_state(src_state)
        self.register_event(event)
        self.register_state(dst_state)

        def get_state_obj(state):
            return state if isinstance(state, State) else self.get_state(state)

        def get_callable(func):
            return func if callable(func) else getattr(self._model, func, None)

        src_state_obj = get_state_obj(src_state)
        dst_state_obj = get_state_obj(dst_state)

        guard_func = get_callable(guard) if guard else None
        action_func = get_callable(action) if action else None
        self.transition_table[(src_state_obj.name, event)] = (
            dst_state_obj,
            guard_func,
            action_func,
        )

    def state_transition(self, src_state: State, event: str):
        if (src_state.name, event) not in self.transition_table:
            raise ValueError(
                f"|{self.name}| invalid transition: <{src_state.name}> : [{event}]"
            )

        dst_state, guard, action = self.transition_table[(src_state.name, event)]

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
                    f"|{self.name}| transitioning from <{src_state.name}> to <{dst_state.name}>"
                )
                src_state.exit()
                self._state = dst_state
                dst_state.enter()
        else:
            print(
                f"|{self.name}| skipping transition from <{src_state.name}> to <{dst_state.name}> because guard failed"
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

        Raises:
            ValueError: If a state with the same name is already registered with a different type.
        """
        if isinstance(state, str):
            if on_enter is None:
                on_enter = getattr(self._model, "on_enter_" + state, None)
            if on_exit is None:
                on_exit = getattr(self._model, "on_exit_" + state, None)
            self.states[state] = State(state, on_enter, on_exit)
            return

        name = state.name
        if name in self.states and type(self.states[name]) is not type(state):
            raise ValueError(
                f'State "{name}" {type(state).__name__} already registered as {type(self.states[name]).__name__}'
            )

        self.states[name] = state

    def register_event(self, event: str):
        self.events[event] = event

    def get_state(self, name):
        return self.states[name]

    def get_event(self, name):
        return self.events[name]

    def has_event(self, event: str):
        return event in self.events

    def set_current_state(self, state: State | str):
        if isinstance(state, str):
            self._state = self.get_state(state)
        else:
            self._state = state

    def get_current_state(self):
        return self._state

    def process(self, event: str) -> None:
        """Process an event in the state machine.

        Args:
            event: Event name or Event object
        """
        if self._state is None:
            raise ValueError("State machine is not initialized")

        if self.has_event(event):
            self.state_transition(self._state, event)
        else:
            raise ValueError(f"Invalid event: {event}")
