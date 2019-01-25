import numpy

class FootstepPlanner(object):
    def __init__(self):
        pass

    def plan(self, goal):
        pass
    
class BipedalPlanner(object):
    def __init__(self):
        self.footstep_planner = FootstepPlanner()

        '''
        self.footstep = [[0.0, 0.2],
                  [0.3, 0.2],
                  [0.3, 0.2],
                  [0.3, 0.2],
                  [0.3, 0.2],
                  [0.0, 0.2]]
        '''

    def set_footstep(self, footstep):
        self.footstep = footstep

    def send_goal(self, goal):
        footstep = self.footstep_planner.plan(goal)
        if footstep != None:
            self.footstep = footstep

if __name__ == "__main__":
    bipedal_planner = BipedalPlanner()
    goal = None
    bipedal_planner.send_goal(goal)
