import conftest
import PathPlanning.FlowField.flowfield as flow_field


def test():
    flow_field.show_animation = False
    flow_field.main()


if __name__ == '__main__':
    conftest.run_this_test(__file__)
