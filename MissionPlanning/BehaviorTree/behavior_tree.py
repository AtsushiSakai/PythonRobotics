"""
Behavior Tree

author: Wang Zheng (@Aglargil)

Reference:

- [Behavior Tree](https://en.wikipedia.org/wiki/Behavior_tree_(artificial_intelligence,_robotics_and_control))
"""

import time
import xml.etree.ElementTree as ET
from enum import Enum


class Status(Enum):
    SUCCESS = "success"
    FAILURE = "failure"
    RUNNING = "running"


class NodeType(Enum):
    CONTROL_NODE = "ControlNode"
    ACTION_NODE = "ActionNode"
    DECORATOR_NODE = "DecoratorNode"


class Node:
    """
    Base class for all nodes in a behavior tree.
    """

    def __init__(self, name):
        self.name = name
        self.status = None

    def tick(self) -> Status:
        """
        Tick the node.

        Returns:
            Status: The status of the node.
        """
        raise ValueError("Node is not implemented")

    def tick_and_set_status(self) -> Status:
        """
        Tick the node and set the status.

        Returns:
            Status: The status of the node.
        """
        self.status = self.tick()
        return self.status

    def reset(self):
        """
        Reset the node.
        """
        self.status = None

    def reset_children(self):
        """
        Reset the children of the node.
        """
        pass


class ControlNode(Node):
    """
    Base class for all control nodes in a behavior tree.

    Control nodes manage the execution flow of their child nodes according to specific rules.
    They typically have multiple children and determine which children to execute and in what order.
    """

    def __init__(self, name):
        super().__init__(name)
        self.children = []
        self.type = NodeType.CONTROL_NODE

    def not_set_children_raise_error(self):
        if len(self.children) == 0:
            raise ValueError("Children are not set")

    def reset_children(self):
        for child in self.children:
            child.reset()


class SequenceNode(ControlNode):
    """
    Executes child nodes in sequence until one fails or all succeed.

    Returns:
        - Returns FAILURE if any child returns FAILURE
        - Returns SUCCESS when all children have succeeded
        - Returns RUNNING when a child is still running or when moving to the next child

    Example:
        .. code-block:: xml

            <Sequence>
                <Action1 />
                <Action2 />
            </Sequence>
    """

    def __init__(self, name):
        super().__init__(name)
        self.current_child_index = 0

    def tick(self) -> Status:
        self.not_set_children_raise_error()

        if self.current_child_index >= len(self.children):
            self.reset_children()
            return Status.SUCCESS
        status = self.children[self.current_child_index].tick_and_set_status()
        if status == Status.FAILURE:
            self.reset_children()
            return Status.FAILURE
        elif status == Status.SUCCESS:
            self.current_child_index += 1
            return Status.RUNNING
        elif status == Status.RUNNING:
            return Status.RUNNING
        else:
            raise ValueError("Unknown status")


class SelectorNode(ControlNode):
    """
    Executes child nodes in sequence until one succeeds or all fail.

    Returns:
        - Returns SUCCESS if any child returns SUCCESS
        - Returns FAILURE when all children have failed
        - Returns RUNNING when a child is still running or when moving to the next child

    Examples:
        .. code-block:: xml

            <Selector>
                <Action1 />
                <Action2 />
            </Selector>
    """

    def __init__(self, name):
        super().__init__(name)
        self.current_child_index = 0

    def tick(self) -> Status:
        self.not_set_children_raise_error()

        if self.current_child_index >= len(self.children):
            self.reset_children()
            return Status.FAILURE
        status = self.children[self.current_child_index].tick_and_set_status()
        if status == Status.SUCCESS:
            self.reset_children()
            return Status.SUCCESS
        elif status == Status.FAILURE:
            self.current_child_index += 1
            return Status.RUNNING
        elif status == Status.RUNNING:
            return Status.RUNNING
        else:
            raise ValueError("Unknown status")


class WhileDoElseNode(ControlNode):
    """
    Conditional execution node with three parts: condition, do, and optional else.

    Returns:
        First executes the condition node (child[0])
        If condition succeeds, executes do node (child[1]) and returns RUNNING
        If condition fails, executes else node (child[2]) if present and returns result of else node
        If condition fails and there is no else node, returns SUCCESS

    Example:
        .. code-block:: xml

            <WhileDoElse>
                <Condition />
                <Do />
                <Else />
            </WhileDoElse>
    """

    def __init__(self, name):
        super().__init__(name)

    def tick(self) -> Status:
        if len(self.children) != 3 and len(self.children) != 2:
            raise ValueError("WhileDoElseNode must have exactly 3 or 2 children")

        condition_node = self.children[0]
        do_node = self.children[1]
        else_node = self.children[2] if len(self.children) == 3 else None

        condition_status = condition_node.tick_and_set_status()
        if condition_status == Status.SUCCESS:
            do_node.tick_and_set_status()
            return Status.RUNNING
        elif condition_status == Status.FAILURE:
            if else_node is not None:
                else_status = else_node.tick_and_set_status()
                if else_status == Status.SUCCESS:
                    self.reset_children()
                    return Status.SUCCESS
                elif else_status == Status.FAILURE:
                    self.reset_children()
                    return Status.FAILURE
                elif else_status == Status.RUNNING:
                    return Status.RUNNING
                else:
                    raise ValueError("Unknown status")
            else:
                self.reset_children()
                return Status.SUCCESS
        else:
            raise ValueError("Unknown status")


class ActionNode(Node):
    """
    Base class for all action nodes in a behavior tree.

    Action nodes are responsible for performing specific tasks or actions.
    They do not have children and are typically used to execute logic or operations.
    """

    def __init__(self, name):
        super().__init__(name)
        self.type = NodeType.ACTION_NODE


class SleepNode(ActionNode):
    """
    Sleep node that sleeps for a specified duration.

    Returns:
        Returns SUCCESS after the specified duration has passed
        Returns RUNNING if the duration has not yet passed

    Example:
        .. code-block:: xml

            <Sleep sec="1.5" />
    """

    def __init__(self, name, duration):
        super().__init__(name)
        self.duration = duration
        self.start_time = None

    def tick(self) -> Status:
        if self.start_time is None:
            self.start_time = time.time()
        if time.time() - self.start_time > self.duration:
            return Status.SUCCESS
        return Status.RUNNING


class EchoNode(ActionNode):
    """
    Echo node that prints a message to the console.

    Returns:
        Returns SUCCESS after the message has been printed

    Example:
        .. code-block:: xml

            <Echo message="Hello, World!" />
    """

    def __init__(self, name, message):
        super().__init__(name)
        self.message = message

    def tick(self) -> Status:
        print(self.name, self.message)
        return Status.SUCCESS


class DecoratorNode(Node):
    """
    Base class for all decorator nodes in a behavior tree.

    Decorator nodes modify the behavior of their child node.
    They must have a single child and can alter the status of the child node.
    """

    def __init__(self, name):
        super().__init__(name)
        self.type = NodeType.DECORATOR_NODE
        self.child = None

    def not_set_child_raise_error(self):
        if self.child is None:
            raise ValueError("Child is not set")

    def reset_children(self):
        self.child.reset()


class InverterNode(DecoratorNode):
    """
    Inverter node that inverts the status of its child node.

    Returns:
        - Returns SUCCESS if the child returns FAILURE
        - Returns FAILURE if the child returns SUCCESS
        - Returns RUNNING if the child returns RUNNING

    Examples:
        .. code-block:: xml

            <Inverter>
                <Action />
            </Inverter>
    """

    def __init__(self, name):
        super().__init__(name)

    def tick(self) -> Status:
        self.not_set_child_raise_error()
        status = self.child.tick_and_set_status()
        return Status.SUCCESS if status == Status.FAILURE else Status.FAILURE


class TimeoutNode(DecoratorNode):
    """
    Timeout node that fails if the child node takes too long to execute

    Returns:
        - FAILURE: If the timeout duration has been exceeded
        - Child's status: Otherwise, passes through the status of the child node

    Example:
        .. code-block:: xml

            <Timeout sec="1.5">
                <Action />
            </Timeout>
    """

    def __init__(self, name, timeout):
        super().__init__(name)
        self.timeout = timeout
        self.start_time = None

    def tick(self) -> Status:
        self.not_set_child_raise_error()
        if self.start_time is None:
            self.start_time = time.time()
        if time.time() - self.start_time > self.timeout:
            return Status.FAILURE
        print(f"{self.name} is running")
        return self.child.tick_and_set_status()


class DelayNode(DecoratorNode):
    """
    Delay node that delays the execution of its child node for a specified duration.

    Returns:
        - Returns RUNNING if the duration has not yet passed
        - Returns child's status after the duration has passed

    Example:
        .. code-block:: xml

            <Delay sec="1.5">
                <Action />
            </Delay>
    """

    def __init__(self, name, delay):
        super().__init__(name)
        self.delay = delay
        self.start_time = None

    def tick(self) -> Status:
        self.not_set_child_raise_error()
        if self.start_time is None:
            self.start_time = time.time()
        if time.time() - self.start_time > self.delay:
            return self.child.tick_and_set_status()
        return Status.RUNNING


class ForceSuccessNode(DecoratorNode):
    """
    ForceSuccess node that always returns SUCCESS.

    Returns:
        - Returns RUNNING if the child returns RUNNING
        - Returns SUCCESS if the child returns SUCCESS or FAILURE
    """

    def __init__(self, name):
        super().__init__(name)

    def tick(self) -> Status:
        self.not_set_child_raise_error()
        status = self.child.tick_and_set_status()
        if status == Status.FAILURE:
            return Status.SUCCESS
        return status


class ForceFailureNode(DecoratorNode):
    """
    ForceFailure node that always returns FAILURE.

    Returns:
        - Returns RUNNING if the child returns RUNNING
        - Returns FAILURE if the child returns SUCCESS or FAILURE
    """

    def __init__(self, name):
        super().__init__(name)

    def tick(self) -> Status:
        self.not_set_child_raise_error()
        status = self.child.tick_and_set_status()
        if status == Status.SUCCESS:
            return Status.FAILURE
        return status


class BehaviorTree:
    """
    Behavior tree class that manages the execution of a behavior tree.
    """

    def __init__(self, root):
        self.root = root

    def tick(self):
        """
        Tick once on the behavior tree.
        """
        self.root.tick_and_set_status()

    def reset(self):
        """
        Reset the behavior tree.
        """
        self.root.reset()

    def tick_while_running(self, interval=None, enable_print=True):
        """
        Tick the behavior tree while it is running.

        Args:
            interval (float, optional): The interval between ticks. Defaults to None.
            enable_print (bool, optional): Whether to print the behavior tree. Defaults to True.
        """
        while self.root.tick_and_set_status() == Status.RUNNING:
            if enable_print:
                self.print_tree()
            if interval is not None:
                time.sleep(interval)
        if enable_print:
            self.print_tree()

    def to_text(self, root, indent=0):
        """
        Recursively convert the behavior tree to a text representation.
        """
        current_text = ""
        if root.status == Status.RUNNING:
            # yellow
            current_text = "\033[93m" + root.name + "\033[0m"
        elif root.status == Status.SUCCESS:
            # green
            current_text = "\033[92m" + root.name + "\033[0m"
        elif root.status == Status.FAILURE:
            # red
            current_text = "\033[91m" + root.name + "\033[0m"
        else:
            current_text = root.name
        if root.type == NodeType.CONTROL_NODE:
            current_text = "  " * indent + "[" + current_text + "]\n"
            for child in root.children:
                current_text += self.to_text(child, indent + 2)
        elif root.type == NodeType.DECORATOR_NODE:
            current_text = "  " * indent + "(" + current_text + ")\n"
            current_text += self.to_text(root.child, indent + 2)
        elif root.type == NodeType.ACTION_NODE:
            current_text = "  " * indent + "<" + current_text + ">\n"
        return current_text

    def print_tree(self):
        """
        Print the behavior tree.

        Node print format:
            Action: <Action>
            Decorator: (Decorator)
            Control: [Control]

        Node status colors:
            Yellow: RUNNING
            Green: SUCCESS
            Red: FAILURE
        """
        text = self.to_text(self.root)
        text = text.strip()
        print("\033[94m" + "Behavior Tree" + "\033[0m")
        print(text)
        print("\033[94m" + "Behavior Tree" + "\033[0m")


class BehaviorTreeFactory:
    """
    Factory class for creating behavior trees from XML strings.
    """

    def __init__(self):
        self.node_builders = {}
        # Control nodes
        self.register_node_builder(
            "Sequence",
            lambda node: SequenceNode(node.attrib.get("name", SequenceNode.__name__)),
        )
        self.register_node_builder(
            "Selector",
            lambda node: SelectorNode(node.attrib.get("name", SelectorNode.__name__)),
        )
        self.register_node_builder(
            "WhileDoElse",
            lambda node: WhileDoElseNode(
                node.attrib.get("name", WhileDoElseNode.__name__)
            ),
        )
        # Decorator nodes
        self.register_node_builder(
            "Inverter",
            lambda node: InverterNode(node.attrib.get("name", InverterNode.__name__)),
        )
        self.register_node_builder(
            "Timeout",
            lambda node: TimeoutNode(
                node.attrib.get("name", SelectorNode.__name__),
                float(node.attrib["sec"]),
            ),
        )
        self.register_node_builder(
            "Delay",
            lambda node: DelayNode(
                node.attrib.get("name", DelayNode.__name__),
                float(node.attrib["sec"]),
            ),
        )
        self.register_node_builder(
            "ForceSuccess",
            lambda node: ForceSuccessNode(
                node.attrib.get("name", ForceSuccessNode.__name__)
            ),
        )
        self.register_node_builder(
            "ForceFailure",
            lambda node: ForceFailureNode(
                node.attrib.get("name", ForceFailureNode.__name__)
            ),
        )
        # Action nodes
        self.register_node_builder(
            "Sleep",
            lambda node: SleepNode(
                node.attrib.get("name", SleepNode.__name__),
                float(node.attrib["sec"]),
            ),
        )
        self.register_node_builder(
            "Echo",
            lambda node: EchoNode(
                node.attrib.get("name", EchoNode.__name__),
                node.attrib["message"],
            ),
        )

    def register_node_builder(self, node_name, builder):
        """
        Register a builder for a node

        Args:
            node_name (str): The name of the node.
            builder (function): The builder function.

        Example:
            .. code-block:: python

                factory = BehaviorTreeFactory()
                factory.register_node_builder(
                    "MyNode",
                    lambda node: MyNode(
                        node.attrib.get("name", MyNode.__name__),
                        node.attrib["my_param"],
                    ),
                )
        """
        self.node_builders[node_name] = builder

    def build_node(self, node):
        """
        Build a node from an XML element.

        Args:
            node (Element): The XML element to build the node from.

        Returns:
            BehaviorTree Node: the built node
        """
        if node.tag in self.node_builders:
            root = self.node_builders[node.tag](node)
            if root.type == NodeType.CONTROL_NODE:
                if len(node) <= 0:
                    raise ValueError(f"{root.name} Control node must have children")
                for child in node:
                    root.children.append(self.build_node(child))
            elif root.type == NodeType.DECORATOR_NODE:
                if len(node) != 1:
                    raise ValueError(
                        f"{root.name} Decorator node must have exactly one child"
                    )
                root.child = self.build_node(node[0])
            elif root.type == NodeType.ACTION_NODE:
                if len(node) != 0:
                    raise ValueError(f"{root.name} Action node must have no children")
            return root
        else:
            raise ValueError(f"Unknown node type: {node.tag}")

    def build_tree(self, xml_string):
        """
        Build a behavior tree from an XML string.

        Args:
            xml_string (str): The XML string containing the behavior tree.

        Returns:
            BehaviorTree: The behavior tree.
        """
        xml_tree = ET.fromstring(xml_string)
        root = self.build_node(xml_tree)
        return BehaviorTree(root)

    def build_tree_from_file(self, file_path):
        """
        Build a behavior tree from a file.

        Args:
            file_path (str): The path to the file containing the behavior tree.

        Returns:
            BehaviorTree: The behavior tree.
        """
        with open(file_path) as file:
            xml_string = file.read()
        return self.build_tree(xml_string)


xml_string = """
    <Sequence name="Sequence">
        <Echo name="Echo0" message="Hello, World0!" />
        <Delay name="Delay" sec="1.5">
            <Echo name="Echo1" message="Hello, World1!" />
        </Delay>
        <Echo name="Echo2" message="Hello, World2!" />
    </Sequence>
   """


def main():
    factory = BehaviorTreeFactory()
    tree = factory.build_tree(xml_string)
    tree.tick_while_running()


if __name__ == "__main__":
    main()
