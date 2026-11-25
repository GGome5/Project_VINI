import math
import logging

import numpy as np

logger = logging.getLogger(__name__)


class Node:
    def __init__(self, position: np.ndarray):
        self.position = position
        self.ref = np.append(position, math.radians(0))
        self.connecting_to_end = False
        self.cost = 0
        self.cost_approx = 0
        self.cost_section = 0
        self.children = []
        self.id = 0
        self.parent = 0
        self.reached = False
        self.safe = True
        self.state = np.zeros((12, 1))
        self.time_reached = 0
        self.traj_length = 0
        self.air_perc = 0
        self.water_perc = 0

        self.reward_collected = 0
        self.budget_used = 0
        self.connecting_to_goal = False

    def add_child(self, child_id):
        self.children.append(child_id)

    def remove_child(self, child_id):
        self.children = [child for child in self.children if child!= child_id]

    def set_parent(self, parent_id):
        self.parent = parent_id

    def set_connecting_to_end(self, connecting_to_end):
        self.connecting_to_end = connecting_to_end

    def set_cost(self, cost):
        self.cost = cost

    def set_cost_approx(self, cost_approx):
        self.cost_approx = cost_approx

    def set_id(self, id):
        self.id = id

    def set_safe(self, is_safe):
        self.safe = is_safe

    def set_state(self, state):
        self.state = state

    def set_reached(self, reached):
        self.reached = reached

    def set_ref(self, ref):
        self.ref = ref

    def set_cost_section(self, cost_section):
        self.cost_section = cost_section

    def set_time_reached(self, time_reached):
        self.time_reached = time_reached

    def set_traj_length(self, traj_length):
        self.traj_length = traj_length

    def set_budget_used(self, budget):
        self.budget_used = budget

    def set_traj_percentages(self, air_perc, water_perc):
        if air_perc < 0:
            logger.info(air_perc)
            logger.info(water_perc)
        self.air_perc = air_perc
        self.water_perc = water_perc