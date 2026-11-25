import math
import random as rd
import numpy as np
import cv2

from viz import Visualization
from cost_map import CostMap
from obstacle import Obstacle
from bias_roulett import Roulette
from goal import Goal
from Node import Node
from class_map import Map
import os
import sys
import yaml
import shutil

# Codigo com a politica de goal inviavel antigo

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

# ══════════════════════════════════════════════════════════════════
# NodeTree  —  lightweight tree container that works with Node IDs
# ══════════════════════════════════════════════════════════════════

class NodeTree:
    """
    Holds a set of Node objects indexed by their integer ID.
    Also stores edges as (parent_id, child_id) pairs for visualisation.
    """
    def __init__(self, root_node: Node):
        self.nodes: dict[int, Node] = {root_node.id: root_node}
        self.edges: list[tuple[int, int]] = []
        self.root_id: int = root_node.id

    def add_node(self, node: Node, parent_id: int):
        node.set_parent(parent_id)
        self.nodes[node.id] = node
        parent = self.nodes[parent_id]
        parent.add_child(node.id)
        self.edges.append((parent_id, node.id))

    def get_node(self, node_id: int) -> Node:
        return self.nodes[node_id]

    def nearest(self, position: np.ndarray) -> Node:
        nodes = list(self.nodes.values())
        positions = np.array([n.position for n in nodes])
        norms = np.sum((positions - position) ** 2, axis=1)
        best_node_id = np.argmin(norms)

        return nodes[best_node_id]

    def ancestor_path(self, node_id: int) -> list[Node]:
        """Walk parent pointers from node_id back to root. Returns [root, ..., node]."""
        path = []
        current = self.nodes[node_id]
        while True:
            path.append(current)
            if current.id == self.root_id:
                break
            parent = self.nodes.get(current.parent)
            if parent is None:
                break
            current = parent
        return list(reversed(path))

    @property
    def all_positions(self) -> list[np.ndarray]:
        return [n.position for n in self.nodes.values()]

    @property
    def all_edges(self) -> list[tuple[np.ndarray, np.ndarray]]:
        result = []
        for pid, cid in self.edges:
            p = self.nodes.get(pid)
            c = self.nodes.get(cid)
            if p and c:
                result.append((p.position, c.position))
        return result


# ══════════════════════════════════════════════════════════════════
# Thin wrapper so Visualization.update() can consume our trees
# ══════════════════════════════════════════════════════════════════

class _VizTree:
    """Minimal duck-type that matches what Visualization.update() expects."""
    def __init__(self):
        self.vertexes: list[np.ndarray] = []
        self.edges: list = []   # list of [p1, p2]



def create_goal(data: np.ndarray, idx: int) -> Goal:
        """
        Factory function to create a Goal from a 3-element vector.
        
        Args:
            data: np.ndarray of shape (3,) with [x, y, reward]
        
        Returns:
            Goal object with position (x, y) and reward value
        """
        if len(data) != 3:
            raise ValueError("Goal data must have exactly 3 elements: [x, y, reward]")
        x, y, reward = data
        return Goal(np.array([x, y]), int(reward), int(idx))


# ══════════════════════════════════════════════════════════════════
# RRT planner
# ══════════════════════════════════════════════════════════════════

class Reward_Guided_RRT:
    def __init__(self, max_iter: int, dist: int, scenario: str, budget: float = 0.0):
        with open('scenerys.yaml', 'r') as f:
                    config = yaml.safe_load(f)

        # ── inital configs ────────────────────────────────────────
        self.SCENARIO = scenario
        scenario_data = config[scenario]
        self.MAX_ITER     = max_iter                               # max tree iteration to expand
        self.DIST         = dist                                   # step size
        self.MIN_REACH    = float(scenario_data['reach_distance']) # goal / start reach radius
        self.BUDGET       = 600.0                                   # total travel budget
        self.BOUNDS       = np.array(scenario_data['map_bounds'])
        self.MAX_IMPROV_ITER = 1000
        self.ITER_RET_FACTOR: int = 1
        if self.MIN_REACH == 50:
            self.ITER_RET_FACTOR = 3
        elif self.MIN_REACH == 25:
            self.ITER_RET_FACTOR = 2
        elif self.MIN_REACH == 10:
            self.ITER_RET_FACTOR = 1.5
        self.MAX_ITER_RET = self.MAX_ITER/self.ITER_RET_FACTOR      # safety cap per returning phase

        # ── Image map usage ────────────────────────────────────────
        self.use_map = scenario_data['use_image_for_obstacles']
        self.mapa = None
        if self.use_map:
            parameters = {  'xlim': [self.BOUNDS[0], self.BOUNDS[1]], 
                            'ylim': [self.BOUNDS[2], self.BOUNDS[3]]   
            }
            self.mapa = Map(parameters, img='maps/city_map.jpeg')

        # ── Environment configs ────────────────────────────────────────
        self.underwater_consume = 1.0
        self.air_consume = 1.0
    

        # ── goals and obstacles ────────────────────────────────────
        goals_array    = np.array(scenario_data['goals'])
        obstacles_array = scenario_data['obstacles']
        cost_points = scenario_data['cost_points']
        self.goal_idx = 0
        self.start_pos  = np.array(scenario_data['start'])
        self.ending_pos = np.array(scenario_data['end'])

        
        self.goals: list[Goal] = []

        self.obstacles : list[Obstacle] = []

        if obstacles_array != None:
            for o_data in obstacles_array:
                o = Obstacle(o_data)

                if o.check_collision_goal(self.start_pos,self.MIN_REACH):
                    print(f"ERROR: The start point {self.start_pos} is inside an obstacle. Stopping execution.")
                    import sys
                    sys.exit(1)

                self.obstacles.append(o)

        for g_data in goals_array:
            g = create_goal(g_data, self.goal_idx)

            if obstacles_array!= None:
                # Verifica se há colisão com QUALQUER um dos obstáculos da lista
                in_collision = any(obs.check_collision_goal(g_data[:2],self.MIN_REACH) for obs in self.obstacles)

                if in_collision:
                    print(f"ERROR: Collision at goal {g_data}.")
                    import sys
                    sys.exit(1)
                else:
                    # Adiciona o goal APENAS UMA VEZ após validar contra todos os obstáculos
                    self.goals.append(g)
                    self.goal_idx += 1
            else:
                self.goals.append(g)
                self.goal_idx += 1


        # ── cost map ────────────────────────────────────
        self.cost_map = CostMap(base_cost=1.0)

        if cost_points != None:     
            for data in cost_points:
                self.cost_map.add_gaussian_cost(data[0],data[1],data[2],data[3])

        # ── global node ID counter ─────────────────────────────────
        self._node_counter = 0

        # Place holder para os nodes adicionados no caminho final

        self.end_nodes = []

        # ── goal tracking ──────────────────────────────────────────
        self.unfeasible_goals: list[Goal] = []
        self.reached_goals:    list[Goal] = []

        # ── roulettes ──────────────────────────────────────────────
        # Outgoing: biased toward all goals, 20% random
        self.out_roulette = Roulette(self.goals,self.cost_map,self.BOUNDS)

        # Returning: only targets the start (20% random, 80% start)
        self._start_goal  = Goal(self.start_pos.copy(), 1, 0)
        self._end_goal    = Goal(self.ending_pos.copy(), 1, 1)
        self.ret_roulette = Roulette([self._end_goal],self.cost_map,self.BOUNDS)

        # ── trees ──────────────────────────────────────────────────
        self.out_tree: NodeTree = self._new_out_tree(self.start_pos, 0.0)
        self.ret_tree: NodeTree | None = None

        # ── solution state ─────────────────────────────────────────
        # going_path: list of legs, each leg is a list of Nodes
        self.going_path:     list[list[Node]] = []
        self.returning_path: list[Node]       = []
        self.all_outward_tree: NodeTree | None = None

        # ── phase control ──────────────────────────────────────────
        self._returning_phase       = False
        self._pending_goal:    Goal | None       = None
        self._pending_segment: list[Node]        = []

        self._iter_out = 0
        self._iter_ret = 0
        self._ret_consec_fail = 0
        self._ret_consec_fail_max = 50
        self.done      = False

        # ── visualisation wrappers ─────────────────────────────────
        self._viz_out = _VizTree()
        self._viz_ret = _VizTree()
        self._viz_all_out = _VizTree()
        self.trees    = [self._viz_out, self._viz_ret]

        # ── data aquisition variables ──────────────────────────────
        self.total_budget_used = 0
        self.total_reward_collected = 0
        self.total_reward_available = 0

    # ── Node / Tree factories ─────────────────────────────────────

    def _make_node(self, position: np.ndarray, budget_used: float,
                   reward_collected: float = 0.0) -> Node:
        n = Node(position.copy())
        n.set_id(self._node_counter)
        n.set_budget_used(budget_used)
        n.reward_collected = reward_collected
        self._node_counter += 1
        return n

    def _new_out_tree(self, root_pos: np.ndarray, budget_so_far: float,
                      reward_so_far: float = 0.0) -> NodeTree:
        root = self._make_node(root_pos, budget_so_far, reward_so_far)
        self.unfeasible_goals = []
        return NodeTree(root)

    def _new_ret_tree(self, root_pos: np.ndarray, budget_so_far: float,
                      reward_so_far: float = 0.0) -> NodeTree:
        root = self._make_node(root_pos, budget_so_far, reward_so_far)
        return NodeTree(root)

    # ── Geometry helpers ──────────────────────────────────────────

    @staticmethod
    def _steer(frm: np.ndarray, to: np.ndarray, dist: float) -> np.ndarray:
        v = to - frm
        length = np.linalg.norm(v)
        if length < 1e-9:
            return frm.copy()
        return frm + (v / length) * min(dist, length)

    def _collision(self, p1: np.ndarray, p2: np.ndarray) -> bool:
        """Checks if a line segment crosses any black obstacle pixels by stepping along it."""
        if self.use_map:
            # Distância Euclidiana entre os dois pontos
            dist = np.linalg.norm(p2 - p1)
            if dist == 0:
                return self.mapa.collision(p1)
                
            # Calcula o número de passos necessários para cobrir a distância com a resolução desejada
            num_steps = max(int(np.ceil(dist / self.DIST)), 2)
            
            # Amostra pontos ao longo da reta
            x_coords = np.linspace(p1[0], p2[0], num_steps)
            y_coords = np.linspace(p1[1], p2[1], num_steps)
            
            # Varre os pontos intermediários
            for x, y in zip(x_coords, y_coords):
                if self.mapa.collision([x, y]):
                    return True  # Colisão encontrada
        else:
            for obs in self.obstacles:
                if obs.check_collision_line(p1, p2):
                    return True
        return False

    def _in_bounds(self, pos: np.ndarray) -> bool:
        xmin, xmax, ymin, ymax = self.BOUNDS
        return xmin <= pos[0] <= xmax and ymin <= pos[1] <= ymax

    # ── Multi environment cost ────────────────────────────────────

    # def _travel_cost(self, q_new: np.ndarray, q_parent: np.ndarray):
    #     p1 = q_parent
    #     p2 = q_new
        
    #     # Distância total entre os dois nós
    #     dist = float(np.linalg.norm(p2 - p1))

    #     y1 = p1[1]
    #     y2 = p2[1]

    #     # Caso 1: Ambos os pontos estão no ar (y >= 500)
    #     if y1 >= 500 and y2 >= 500:
    #         return dist * self.air_consume

    #     # Caso 2: Ambos os pontos estão na água (y <= 500)
    #     elif y1 <= 500 and y2 <= 500:
    #         return dist * self.underwater_consume

    #     # Caso 3: Transição entre os meios (cruza o eixo y = 500)
    #     else:
    #         # Usando semelhança de triângulos, calculamos a fração da reta que está de cada lado
    #         total_y_diff = abs(y1 - y2)
            
    #         # Distância percorrida em relação ao ponto 1 e ponto 2 até a superfície (y=500)
    #         dist_p1_to_surface = dist * (abs(y1 - 500) / total_y_diff)
    #         dist_p2_to_surface = dist * (abs(y2 - 500) / total_y_diff)

    #         # Se o ponto 1 estava no ar (y1 > 0) e o ponto 2 na água
    #         if y1 > 500:
    #             cost_air   = dist_p1_to_surface * self.air_consume
    #             cost_water = dist_p2_to_surface * self.underwater_consume
    #             #print("Indo do ar para a agua")
    #         # Se o ponto 1 estava na água (y1 < 0) e o ponto 2 no ar
    #         else:
    #             cost_water = dist_p1_to_surface * self.underwater_consume
    #             cost_air   = dist_p2_to_surface * self.air_consume
    #             #print("Indo da agua para o ar")

    #         return cost_air + cost_water


    # ── Sampling ──────────────────────────────────────────────────

    def _sample_out(self) -> np.ndarray:
        excluded = [tuple(g.pos) for g in self.unfeasible_goals + self.reached_goals]
        root_node = self.out_tree.get_node(self.out_tree.root_id)
        root = root_node.position

        goal_pos = self.out_roulette.spin(excluded, root)

        return goal_pos.astype(float)

    def _sample_ret(self) -> np.ndarray:
        # empty reached list so start is always available
        root_node = self.out_tree.get_node(self.out_tree.root_id)
        root = root_node.position
        goal_pos = self.ret_roulette.spin([], root)

        return goal_pos.astype(float)

     # ── RRT* functions ───────────────────────────────────────────

    def _get_neighbors_in_radius(self, q_new: np.ndarray, tree: NodeTree) -> list:
        nodes_in_radius = []

        area = (self.BOUNDS[1] - self.BOUNDS[0]) * (self.BOUNDS[3] - self.BOUNDS[2])
        d = 2
        n = len(tree.nodes)

        gamma = 4.0 * ((1 + 1/d) * (area/math.pi)) ** (1/d)
        # Enforce a minimum radius of 2*DIST so rewiring stays effective
        # even when the tree is large (the pure formula collapses toward 0).
        radius = min(gamma * (math.log(n)/n) ** (1/d), self.DIST * 2)

        for node in tree.nodes.values():
            dist = np.linalg.norm(node.position - q_new)
            if dist <= radius:
                nodes_in_radius.append(node)
        return nodes_in_radius

    def _choose_parent(self, neighbors: list, nearest: Node, q_new: np.ndarray) -> Node:
        candidates = list(neighbors)
        if nearest not in candidates:
            candidates.append(nearest)

        # Calcula custos para todos sem verificar colisao ainda
        ranked = []
        for neighbor in candidates:
            c_x = neighbor.budget_used + self.cost_map.travel_cost(q_new, neighbor.position)
            #c_x = neighbor.budget_used + float(np.linalg.norm(q_new - neighbor.position))
            ranked.append((c_x, -neighbor.reward_collected, neighbor))

        # Ordena: menor custo primeiro, maior reward em empate
        ranked.sort(key=lambda item: (item[0], item[1]))

        # Verifica colisao apenas ate encontrar o primeiro valido
        for c_x, neg_reward, neighbor in ranked:
            if self._collision(neighbor.position, q_new):
                continue
            return neighbor

        return nearest  # fallback

    def _rewire(self, tree: NodeTree, neighbors: list, q_min: Node, q_new: Node) -> NodeTree:
        for neighbor in neighbors:
            if neighbor.id == q_min.id or neighbor.id == tree.root_id:
                continue
            edge      = self.cost_map.travel_cost(q_new.position, neighbor.position)    
            #edge     = self._travel_cost(q_new.position, neighbor.position)
            #edge     = float(np.linalg.norm(neighbor.position - q_new.position))
            new_cost = q_new.budget_used + edge

            cost_improves   = new_cost < neighbor.budget_used
            reward_improves = (new_cost <= self.BUDGET and
                            q_new.reward_collected > neighbor.reward_collected)

            if not (cost_improves or reward_improves):
                continue

            # Ancestor check: walk from q_new upward to avoid cycles
            is_ancestor = False
            check_node  = q_new
            while check_node.id != tree.root_id:
                if check_node.parent == neighbor.id:
                    is_ancestor = True
                    break
                parent = tree.nodes.get(check_node.parent)
                if parent is None:
                    break
                check_node = parent

            if is_ancestor:
                continue

            if self._collision(q_new.position, neighbor.position):
                continue

            # Detach neighbor from its old parent
            old_parent = tree.nodes.get(neighbor.parent)
            if old_parent is not None:
                if neighbor.id in old_parent.children:        # verificacao segura
                    old_parent.children.remove(neighbor.id)
            if (neighbor.parent, neighbor.id) in tree.edges:
                tree.edges.remove((neighbor.parent, neighbor.id))

            # Attach neighbor to q_new
            neighbor.set_parent(q_new.id)
            q_new.add_child(neighbor.id)
            tree.edges.append((q_new.id, neighbor.id))

            # Update cost and reward, then propagate to entire subtree
            neighbor.set_budget_used(new_cost)
            neighbor.reward_collected = q_new.reward_collected
            self._propagate_cost(tree, neighbor)

        return tree

    def _propagate_cost(self, tree: NodeTree, parent: Node):
        """Recursively update budget_used and reward_collected for all descendants."""
        for child_id in parent.children:
            child = tree.nodes[child_id]
            edge  = self.cost_map.travel_cost(child.position, parent.position)
            #edge = self._travel_cost(child.position, parent.position)
            #edge  = float(np.linalg.norm(child.position - parent.position))
            child.set_budget_used(parent.budget_used + edge)
            child.reward_collected = parent.reward_collected
            self._propagate_cost(tree, child)

    # ══════════════════════════════════════════════════════════════
    # Main step — called once per timer tick
    # ══════════════════════════════════════════════════════════════

    def step(self) -> bool:
        if self.done:
            self._refresh_viz()
            return True

        if self._returning_phase:
            self._step_return()
        else:
            self._step_outgoing()

        self._refresh_viz()
        return self.done

    # ── Outgoing expansion ────────────────────────────────────────

    def _step_outgoing(self):
        if self._iter_out >= self.MAX_ITER:
            #print("Outgoing tree hit iteration cap without finding a new goal.")
            self._merge_outward_tree()
            self.done = True
            return

        self._iter_out += 1

        q_rand  = self._sample_out()
        nearest = self.out_tree.nearest(q_rand)
        q_new   = self._steer(nearest.position, q_rand, self.DIST)

        if not self._in_bounds(q_new):
            return
        if self._collision(nearest.position, q_new):
            return

        nodes_in_radius = self._get_neighbors_in_radius(q_new, self.out_tree)
        parent = self._choose_parent(nodes_in_radius, nearest, q_new)

        edge_cost = self.cost_map.travel_cost(q_new, parent.position)
        #edge_cost = self._travel_cost(q_new, parent.position)
        #edge_cost  = float(np.linalg.norm(q_new - parent.position))
        new_budget = parent.budget_used + edge_cost

        if new_budget >= self.BUDGET:
            # This branch is exhausted; check if all remaining goals
            # are now unreachable (handled implicitly by iter cap)
            #print("!!!!!!! Excedeu o limite de budget nesse galho !!!!!!")
            return

        # Inherit running reward from parent; goal reward added below if touched
        new_node = self._make_node(q_new, new_budget, parent.reward_collected)
        self.out_tree.add_node(new_node, parent.id)

        self.out_tree = self._rewire(self.out_tree, nodes_in_radius, parent, new_node)

        # Check if we reached any available goal
        for goal in self.goals:
            if goal in self.unfeasible_goals or goal in self.reached_goals:
                continue
            if np.linalg.norm(goal.pos - q_new) <= self.MIN_REACH:
                # Add this goal's reward to the touching node
                new_node.reward_collected += goal.reward
                new_node.connecting_to_goal = True
                # print(f"[OUT] Reached goal {goal.pos}  "
                #       f"budget so far: {new_budget:.2f}/{self.BUDGET}  "
                #       f"reward so far: {new_node.reward_collected:.0f}")
                self._start_return_phase(new_node, goal)
                #return   # returning phase takes over

    # ── Return expansion ──────────────────────────────────────────

    def _step_return(self):
        if self._iter_ret >= self.MAX_ITER_RET:
            #print(f"[RET] Hit iteration cap — goal {self._pending_goal.pos} unfeasible.")
            self._fail_return()
            return

        self._iter_ret += 1

        q_rand  = self._sample_ret()
        nearest = self.ret_tree.nearest(q_rand)
        q_new   = self._steer(nearest.position, q_rand, self.DIST)

        if not self._in_bounds(q_new):
            return
        if self._collision(nearest.position, q_new):
            return

        nodes_in_radius = self._get_neighbors_in_radius(q_new, self.ret_tree)
        parent = self._choose_parent(nodes_in_radius, nearest, q_new)

        edge_cost = self.cost_map.travel_cost(q_new,parent.position)
        # edge_cost = self._travel_cost(q_new, parent.position)
        # edge_cost  = float(np.linalg.norm(q_new - parent.position))
        new_budget = parent.budget_used + edge_cost

        '''
        Possivel causa da alta taxa de falha de retorno.
        Assim que o budget estoura ele ja confere como enviavel, preciso ver novas politicas de rejeitar um goal.
        '''

        if self._ret_consec_fail >= self._ret_consec_fail_max:
            print(f"Ret tree failed {self._ret_consec_fail} times")
            self._fail_return()
            self._ret_consec_fail = 0
            return

        if new_budget >= self.BUDGET:
            # print(f"[RET] Budget exhausted — goal {self._pending_goal.pos} unfeasible.")
            # print(new_budget)
            self._ret_consec_fail += 1
            return

        self._ret_consec_fail = 0


        new_node = self._make_node(q_new, new_budget, parent.reward_collected)
        self.ret_tree.add_node(new_node, parent.id)

        self.ret_tree = self._rewire(self.ret_tree, nodes_in_radius, parent, new_node)

        excluded_in_return = (
            self.reached_goals +
            self.unfeasible_goals +
            [self._pending_goal]
        )

        # Check if we reached START
        if np.linalg.norm(q_new - self.ending_pos) <= self.MIN_REACH:
            # print(f"[RET] Reached START!  "
            #       f"Total budget used: {new_budget:.2f}/{self.BUDGET}")
            self._confirm_return(new_node)

    # ── Phase transitions ─────────────────────────────────────────

    def _start_return_phase(self, goal_node: Node, goal: Goal):
        self._returning_phase  = True
        self._pending_goal     = goal
        self._iter_ret         = 0

        # Return tree roots at the goal node position,
        # carrying all budget accumulated so far
        self.ret_tree = self._new_ret_tree(goal_node.position, goal_node.budget_used)

        # Save going segment tentatively (committed only if return succeeds)
        self._pending_segment = self.out_tree.ancestor_path(goal_node.id)

    def _confirm_return(self, ret_end_node: Node):
        goal = self._pending_goal

        # Total reward accumulated at the goal node that closed this leg
        confirmed_reward = self._pending_segment[-1].reward_collected

        # Back-fill reward_collected onto every node in the going segment
        # so the full path consistently reflects the confirmed total
        for node in self._pending_segment:
            node.reward_collected = confirmed_reward

        # Commit going segment
        self.going_path.append(self._pending_segment)
        self.reached_goals.append(goal)

        # Replace returning path with the new one
        self.returning_path = self.ret_tree.ancestor_path(ret_end_node.id)

        # print(f"✓ Round trip confirmed via {goal.pos}  "
        #       f"| legs so far: {len(self.going_path)}  "
        #       f"| total reward: {confirmed_reward:.0f}")
        
        self._greedy_path_shortcut()
        self._two_opt()

        # Restart outgoing tree from the last node of the going segment,
        # carrying both budget and reward accumulated so far
        last_node     = self._pending_segment[-1]
        self._merge_outward_tree()
        self.out_tree = self._new_out_tree(last_node.position,
                                           last_node.budget_used,
                                           last_node.reward_collected)
        self._iter_out = 0

        # Reset return state
        self._returning_phase = False
        self._pending_goal    = None
        self._pending_segment = []
        self.ret_tree         = None

        # Done if no goals remain
        remaining = [g for g in self.goals
                     if g not in self.reached_goals and g not in self.unfeasible_goals]
        if not remaining:
            print("All goals processed. Planning complete.")
            self._merge_outward_tree()
            self.done = True

    def _fail_return(self):
        goal = self._pending_goal
        self.unfeasible_goals.append(goal)
        #print(f"✗ Goal {goal.pos} marked unfeasible in the returning.")

        # Resume outgoing tree — do NOT reset it, just continue expanding
        self._returning_phase = False
        self._pending_goal    = None
        self._pending_segment = []
        self.ret_tree         = None
        self._iter_ret        = 0

        remaining = [g for g in self.goals
                     if g not in self.reached_goals and g not in self.unfeasible_goals]
        if not remaining:
            print("No more goals available. Planning complete.")
            self.done = True

    # ── Visualisation sync ────────────────────────────────────────

    def _refresh_viz(self):
        self._viz_out.edges    = [list(e) for e in self.out_tree.all_edges]
        self._viz_out.vertexes = self.out_tree.all_positions

        if self.ret_tree is not None:
            self._viz_ret.edges    = [list(e) for e in self.ret_tree.all_edges]
            self._viz_ret.vertexes = self.ret_tree.all_positions
        else:
            self._viz_ret.edges    = []
            self._viz_ret.vertexes = []

    # ── Solution summary ──────────────────────────────────────────

    def print_solution(self):
        self._merge_return_tree()
        print("\n== Solution ==")
        if not self.going_path:
            print("No feasible path found.")
            return
        
        # Metodo para a coleta de goals que o caminho de volta passou dentro do raio de coleta

        if self.returning_path:
            for idx, n in enumerate(self.returning_path):
                if idx < len(self.returning_path) - 1:
                    next_n = self.returning_path[idx + 1]

                    distance = np.linalg.norm(n.position - next_n.position)
                    if distance < 1e-9:
                        continue
                    t = np.arange(0, distance + 1, 1.0)
                    discrete_points = n.position + np.outer(
                        t / distance, next_n.position - n.position
                    )

                    for g in self.goals:
                        if g in self.reached_goals:    # exclusao no loop externo
                            continue
                        for p in discrete_points:
                            if np.linalg.norm(g.pos - p) <= self.MIN_REACH:
                                self.reached_goals.append(g)
                                self.end_nodes.append(np.array(p).copy())
                                print(f"Reward coletado no caminho final de volta: "
                                    f"{g.pos[0]} {g.pos[1]} {g.reward}")
                                break

        if self.going_path:
            for subpath in self.going_path:
                for idx, n in enumerate(subpath):
                    if idx < len(subpath) - 1:
                        next_n   = subpath[idx + 1]
                        distance = np.linalg.norm(n.position - next_n.position)

                        if distance < 1e-9:
                            continue

                        t = np.arange(0, distance + 1, 1.0)
                        discrete_points = n.position + np.outer(
                            t / distance, next_n.position - n.position
                        )

                        for g in self.goals:
                            if g in self.reached_goals:    # exclusao no loop externo
                                continue
                            for p in discrete_points:
                                if np.linalg.norm(g.pos - p) <= self.MIN_REACH:
                                    self.reached_goals.append(g)
                                    self.end_nodes.append(np.array(p).copy())
                                    print(f"Reward coletado no caminho final de volta: "
                                        f"{g.pos[0]} {g.pos[1]} {g.reward}")
                                    break
                    

        # Budget spent is the budget_used of the last node of the return path
        budget_spent = self.returning_path[-1].budget_used if self.returning_path else 0.0
        total_reward_collected = sum(g.reward for g in self.reached_goals)
        total_reward_available = sum(g.reward for g in self.goals)

        self.total_budget_used = budget_spent
        self.total_reward_collected = total_reward_collected
        self.total_reward_available = total_reward_available

        # Visitation sequence: START -> goal_A -> goal_B -> ... -> START
        sequence = ["START"] + [str(g.pos.tolist()) for g in self.reached_goals] + ["START"]
        sequence_str = " -> ".join(sequence)

        print(f"  Budget spent    : {budget_spent:.2f} / {self.BUDGET:.2f}  "
              f"({100 * budget_spent / self.BUDGET:.1f}%)")
        print(f"  Reward collected: {total_reward_collected} / {total_reward_available}  "
              f"({100 * total_reward_collected / total_reward_available:.1f}%)")
        print(f"  Visitation order: {sequence_str}")
        print(f"  Goals unfeasible: {[str(g.pos.tolist()) for g in self.unfeasible_goals]}")

        print("\nGoing path legs:")
        for i, seg in enumerate(self.going_path):
            print(f"  Leg {i+1}: {len(seg)} nodes  "
                  f"{seg[0].position} -> {seg[-1].position}  "
                  f"budget at end = {seg[-1].budget_used:.2f}  "
                  f"reward at end = {seg[-1].reward_collected:.0f}")

        if self.returning_path:
            last = self.returning_path[-1]
            print(f"\nReturning path: {len(self.returning_path)} nodes  "
                  f"budget at end = {last.budget_used:.2f}/{self.BUDGET}")


    # -- Final path visualisation ---------------------------------

    def show_final_path(self):
        """
        Flatten going_path legs + returning_path into two _VizTree objects
        and replace self.trees so the next plot.update() call renders only
        the confirmed solution path, not the full exploration trees.
        """
        viz_going  = _VizTree()
        viz_return = _VizTree()

        # Going path: chain all legs in order.
        # Consecutive legs share a junction position so skip the first
        # node of each leg after the first to avoid duplicate edges.
        all_going_nodes = []
        for i, seg in enumerate(self.going_path):
            # all_going_nodes.extend(seg if i == 0 else seg[1:])
            all_going_nodes.extend(seg)

        viz_going.vertexes = [n.position for n in all_going_nodes]
        viz_going.edges    = [
            [all_going_nodes[i].position, all_going_nodes[i + 1].position]
            for i in range(len(all_going_nodes) - 1)
        ]

        # Returning path
        viz_return.vertexes = [n.position for n in self.returning_path]
        viz_return.edges    = [
            [self.returning_path[i].position, self.returning_path[i + 1].position]
            for i in range(len(self.returning_path) - 1)
        ]

        # Swap live exploration trees for the two final path trees
        self.trees = [viz_going, viz_return]

    def print_goals_list(self):
        for g in self.goals:
            print(f"[{g.pos[0]}, {g.pos[1]}, {g.reward}],")

    # ── 2opt optimization ──────────────────────────────────────────


    def _two_opt(self):
        """
        Apply 2-opt to the going_path.
    
        For each node in order (starting from the first in the list):
        - Find the best node to connect to (most budget improvement, collision-free)
        - If found: apply classic 2-opt reversal of the segment between them
        - Move to the next position (the node just placed after current)
        - If not found: move to the next node
        Stops when all nodes have been processed.
        """
        if not self.going_path:
            return
    
        #print(f"[2-OPT] Starting...")
    
        # Flatten going_path and track segment boundaries for reconstruction
        all_nodes = []
        segment_boundaries = []
        for seg in self.going_path:
            all_nodes.extend(seg)
            segment_boundaries.append(len(all_nodes) - 1)
    
        # Single forward pass: for each node find best swap, apply and move forward
        total_swaps = 0
        current = 0
    
        while current < len(all_nodes) - 2:
            best_j = None
            best_improvement = 0.0
    
            for j in range(current + 2, len(all_nodes) - 1):
                curr = all_nodes[current]
                next_curr = all_nodes[current + 1]
                j_node = all_nodes[j]
                next_j = all_nodes[j + 1]

                new_cost = self.cost_map.travel_cost(curr.position, j_node.position) + self.cost_map.travel_cost(next_curr.position, next_j.position)
                old_cost = self.cost_map.travel_cost(curr.position, next_curr.position) + self.cost_map.travel_cost(j_node.position, next_j.position)

                # new_cost = self._travel_cost(curr.position, j_node.position) + self._travel_cost(next_curr.position, next_j.position)
                # old_cost = self._travel_cost(curr.position, next_curr.position) + self._travel_cost(j_node.position, next_j.position)
    
                # new_cost = float(np.linalg.norm(curr.position - j_node.position)) + float(np.linalg.norm(next_curr.position - next_j.position)) 
                # old_cost = float(np.linalg.norm(curr.position - next_curr.position)) + float(np.linalg.norm(j_node.position - next_j.position))
    
                improvement = old_cost - new_cost
                if improvement > best_improvement:
                    if self._collision(curr.position, j_node.position) or self._collision(next_curr.position, next_j.position):
                        continue

                    best_improvement = improvement
                    best_j = j
    
            if best_j is not None:
                # Classic 2-opt: reverse segment between current+1 and best_j (inclusive)
                all_nodes[current + 1:best_j + 1] = all_nodes[current + 1:best_j + 1][::-1]
                # print(f"  [2-OPT] current={current} -> best_j={best_j}  "
                #     f"improvement={best_improvement:.2f}")
                total_swaps += 1
    
            # Always advance: move to next node regardless of swap
            # (ignore the last rearranged node on next iteration)
            current += 1
    
        # Recompute budget_used from the first rearranged index onwards
        if total_swaps > 0:
            # start = max(1, indices_in_radius[0])
            for i in range(1, len(all_nodes)):
                prev = all_nodes[i - 1]
                curr = all_nodes[i]
                edge = self.cost_map.travel_cost(curr.position, prev.position)
                # edge = self._travel_cost(curr.position, prev.position)
                #edge = float(np.linalg.norm(curr.position - prev.position))
                curr.set_budget_used(prev.budget_used + edge)
    
        # Rebuild going_path segments from the updated all_nodes
        self.going_path = []
        prev = 0
        for boundary in segment_boundaries:
            self.going_path.append(all_nodes[prev:boundary + 1])
            prev = boundary + 1
    
        # print(f"[2-OPT] Done. {total_swaps} swaps applied.")

    

    # ── Greedy path optimization ──────────────────────────────────────────

    def _greedy_path_shortcut(self):
        """Greedily shortcut going and returning paths separately."""
        # print(f"[SHORTCUT] Starting path optimization...")
    
        for seg_idx in range(len(self.going_path)):
            self.going_path[seg_idx] = self._shortcut_segment(
                self.going_path[seg_idx], label=f"OUT-{seg_idx}"
            )
    
        self.returning_path = self._shortcut_segment(
            self.returning_path, label="RET"
        )
    
        # print(f"[SHORTCUT] Done.")
    
    def _shortcut_segment(self, segment: list, label: str = "") -> list:
        """
        Greedily shortcut a path segment.
    
        Reverse the segment to start from the last node.
        Iterate forward, trying to skip intermediate nodes when
        the direct edge is shorter. Collect skipped nodes in
        temp_removed; when no further improvement is possible,
        flush temp_removed into all_to_remove, advance by the
        number of removed nodes, and clear temp_removed.
        Finally, rebuild the segment excluding all_to_remove.
        """
        if len(segment) < 3:
            return segment
    
        reversed_seg = list(reversed(segment))
        all_to_remove = []
    
        i = 0
        while i < len(reversed_seg) - 1:
            temp_removed = []
            j = i + 2  # j=i+1 is the direct neighbor; start at i+2 to try skipping i+1
    
            while j < len(reversed_seg):
                current = reversed_seg[i]
                target  = reversed_seg[j]

                if reversed_seg[j - 1].connecting_to_goal:
                    break

                new_cost = self.cost_map.travel_cost(current.position, target.position)
                # new_cost = self._travel_cost(current.position, target.position)
                #new_cost = float(np.linalg.norm(current.position - target.position))
                old_cost = current.budget_used - target.budget_used
    
                if new_cost <= old_cost + 0.01 and not self._collision(current.position, target.position):
                    # Node at j-1 is being skipped
                    temp_removed.append(reversed_seg[j - 1])
                    j += 1
                else:
                    break
    
            # if temp_removed:
            #     print(f"  [{label}] i={i}: skipping {len(temp_removed)} nodes")
    
            all_to_remove.extend(temp_removed)
            i += len(temp_removed) + 1  # Jump past removed nodes to continue
            temp_removed = []            # Clear for next iteration
    
        # Rebuild segment excluding removed nodes
        to_remove_ids = {id(n) for n in all_to_remove}
        new_segment = [n for n in segment if id(n) not in to_remove_ids]
    
        # Recompute budget_used for the shortened segment
        for k in range(1, len(new_segment)):
            prev = new_segment[k - 1]
            curr = new_segment[k]
            edge = self.cost_map.travel_cost(curr.position, prev.position)
            # edge = self._travel_cost(curr.position, prev.position)
            #edge = float(np.linalg.norm(curr.position - prev.position))
            curr.set_budget_used(prev.budget_used + edge)
    
        # print(f"  [{label}] {len(all_to_remove)} nodes removed, "
        #     f"{len(new_segment)} nodes remain.")
    
        return new_segment

    def _improve_path(self):
        """
        Current not being used, broken.

        Post-processing path improvement.

        For each segment of going_path and returning_path, expands a new RRT
        trying to find a shorter connection between the same two endpoints.
        If the new segment has lower budget cost, it replaces the old one and
        the delta is propagated forward to all subsequent segments and the
        returning_path.

        Goals not already in reached_goals are collected if the new path
        passes within MIN_REACH of them.
        """


        # ── going_path segments ───────────────────────────────────────────
        for seg_idx in range(len(self.going_path)):
            segment    = self.going_path[seg_idx]
            start_node = segment[0]
            end_node   = segment[-1]

            # Find the confirmed goal at the end of this segment
            end_goal = next(
                (g for g in self.reached_goals
                if np.linalg.norm(g.pos - end_node.position) <= self.MIN_REACH),
                None
            )

            if end_goal is None:
                continue

            old_end_budget = end_node.budget_used
            old_seg_cost   = old_end_budget - start_node.budget_used

            # Roulette targeting only the goal at the end of this segment —
            # same behaviour as ret_roulette but toward the end goal
            seg_roulette = Roulette([end_goal])

            # New tree rooted at the start of this segment
            new_tree = NodeTree(
                self._make_node(
                    start_node.position,
                    start_node.budget_used,
                    start_node.reward_collected
                )
            )

            best_end_node = None
            best_cost     = old_seg_cost
            best_raw_cost = np.inf

            for _ in range(self.MAX_IMPROV_ITER):
                goal_pos, is_random = seg_roulette.spin([], start_node.position)

                if is_random or goal_pos is None:
                    xmin, xmax, ymin, ymax = self.BOUNDS
                    q_rand = np.array([rd.uniform(xmin, xmax),
                                    rd.uniform(ymin, ymax)])
                else:
                    q_rand = goal_pos.astype(float)

                nearest = new_tree.nearest(q_rand)
                q_new   = self._steer(nearest.position, q_rand, self.DIST)

                if not self._in_bounds(q_new):
                    continue
                if self._collision(nearest.position, q_new):
                    continue

                nodes_in_radius = self._get_neighbors_in_radius(q_new, new_tree)
                parent          = self._choose_parent(nodes_in_radius, nearest, q_new)

                edge_cost = self.cost_map.travel_cost(q_new,parent.position)
                # edge_cost = self._travel_cost(q_new,parent.position)
                new_budget = parent.budget_used + edge_cost

                if new_budget >= self.BUDGET:
                    continue

                new_node = self._make_node(q_new, new_budget,
                                        parent.reward_collected)
                new_tree.add_node(new_node, parent.id)
                new_tree = self._rewire(new_tree, nodes_in_radius, parent, new_node)

                # Collect intermediate goals not already reached
                # for g in self.goals:
                #     if g in self.reached_goals:
                #         continue
                #     if np.linalg.norm(g.pos - q_new) <= self.MIN_REACH:
                #         new_node.reward_collected += g.reward
                #         self.reached_goals.append(g)
                #         self._propagate_cost(new_tree, new_node)

                # Check if reached the end goal
                if np.linalg.norm(q_new - end_node.position) <= self.MIN_REACH:
                        seg_cost = new_budget - start_node.budget_used
                        if seg_cost < best_raw_cost:
                            best_raw_cost = seg_cost
                            best_end_node = new_node

            if best_end_node is None:
                continue

            # Extract and shortcut the new segment
            new_segment = new_tree.ancestor_path(best_end_node.id)
            new_segment = self._shortcut_segment(new_segment, label=f"IMPROV-OUT-{seg_idx}")

            # Compara o custo APÓS shortcut com o custo do segmento original
            new_seg_cost = new_segment[-1].budget_used - start_node.budget_used

            if new_seg_cost > old_seg_cost:
                print(f"[IMPROV] Segment {seg_idx} not improved")
                continue  # nao melhorou nem apos shortcut, descarta


            # Compute how much the budget changed at the segment boundary
            delta = new_segment[-1].budget_used - old_end_budget

            # Replace the segment
            self.going_path[seg_idx] = new_segment

            # Propagate delta forward to all subsequent going_path segments
            for next_seg in self.going_path[seg_idx + 1:]:
                for node in next_seg:
                    node.set_budget_used(node.budget_used + delta)

            # Propagate delta to returning_path
            for node in self.returning_path:
                node.set_budget_used(node.budget_used + delta)

            print(f"[IMPROV] Segment {seg_idx} improved:  "
                f"old_cost={old_seg_cost:.2f}  "
                f"new_cost={best_cost:.2f}  "
                f"delta={delta:.2f}")

        # ── returning_path ────────────────────────────────────────────────
        if not self.returning_path:
            return

        ret_start          = self.returning_path[0]
        old_ret_end_budget = self.returning_path[-1].budget_used
        old_ret_cost       = old_ret_end_budget - ret_start.budget_used

        # New tree rooted at the start of the returning path
        new_ret_tree = NodeTree(
            self._make_node(
                ret_start.position,
                ret_start.budget_used,
                ret_start.reward_collected
            )
        )

        best_ret_end  = None
        best_ret_cost = old_ret_cost
        best_raw_cost = np.inf

        for _ in range(self.MAX_IMPROV_ITER):
            goal_pos, is_random = self.ret_roulette.spin([], ret_start.position)

            if is_random or goal_pos is None:
                xmin, xmax, ymin, ymax = self.BOUNDS
                q_rand = np.array([rd.uniform(xmin, xmax),
                                rd.uniform(ymin, ymax)])
            else:
                q_rand = goal_pos.astype(float)

            nearest = new_ret_tree.nearest(q_rand)
            q_new   = self._steer(nearest.position, q_rand, self.DIST)

            if not self._in_bounds(q_new):
                continue
            if self._collision(nearest.position, q_new):
                continue

            nodes_in_radius = self._get_neighbors_in_radius(q_new, new_ret_tree)
            parent          = self._choose_parent(nodes_in_radius, nearest, q_new)

            edge_cost  = float(np.linalg.norm(q_new - parent.position))
            new_budget = parent.budget_used + edge_cost

            if new_budget >= self.BUDGET:
                continue

            new_node = self._make_node(q_new, new_budget,
                                    parent.reward_collected)
            new_ret_tree.add_node(new_node, parent.id)
            new_ret_tree = self._rewire(new_ret_tree, nodes_in_radius,
                                        parent, new_node)

            # Both are bugged, do not use
            # Collect intermediate goals not already reached
            # for g in self.goals:
            #     if g in self.reached_goals:
            #         continue
            #     if np.linalg.norm(g.pos - q_new) <= self.MIN_REACH:
            #         new_node.reward_collected += g.reward
            #         self.reached_goals.append(g)
            #         self._propagate_cost(new_ret_tree, new_node)

            # Check if reached START
            if np.linalg.norm(q_new - end_node.position) <= self.MIN_REACH:
                seg_cost = new_budget - ret_start.budget_used
                if seg_cost < best_raw_cost:
                    best_raw_cost = seg_cost
                    best_end_node = new_node

        if best_ret_end is not None:
            new_ret_path = new_ret_tree.ancestor_path(best_ret_end.id)
            new_ret_path = self._shortcut_segment(new_ret_path, label="IMPROV-RET")

            new_seg_cost = new_ret_path[-1].budget_used - ret_start.budget_used

            if new_seg_cost > old_ret_cost:
                print(f"[IMPROV] Return path not improved:")
                return  

            delta = new_ret_path[-1].budget_used - old_ret_end_budget
            self.returning_path = new_ret_path

            print(f"[IMPROV] Return path improved:  "
                f"old_cost={old_ret_cost:.2f}  "
                f"new_cost={best_ret_cost:.2f}  "
                f"delta={delta:.2f}")

    

    def _merge_outward_tree(self):
        """Funde a arvore de ida atual na arvore acumulada."""
        if self.all_outward_tree is None:
            # Primeira arvore — inicializa com o root da arvore atual
            root = self.out_tree.get_node(self.out_tree.root_id)
            self.all_outward_tree = NodeTree(root)

        # Adiciona todos os nodes e edges da arvore atual
        for node_id, node in self.out_tree.nodes.items():
            self.all_outward_tree.nodes[node_id] = node

        for edge in self.out_tree.edges:
            if edge not in self.all_outward_tree.edges:
                self.all_outward_tree.edges.append(edge)

    def _merge_return_tree(self):
        """Funde TODOS os nodes do ret_tree final na arvore acumulada."""
        if self.ret_tree is None:
            return
        if self.all_outward_tree is None:
            return

        # Mescla todos os nodes da arvore de retorno final
        for node_id, node in self.ret_tree.nodes.items():
            self.all_outward_tree.nodes[node_id] = node

        for edge in self.ret_tree.edges:
            if edge not in self.all_outward_tree.edges:
                self.all_outward_tree.edges.append(edge)

    def _genarate_randon_goals(self, n_goals):
        n = 0
        while n < n_goals:
            # 1. Gera a posição (use randint para inteiros ou uniform para floats)
            candidate_pos = np.array([float(rd.uniform(25, 1475)), float(rd.uniform(25, 975.0))])
            
            # 2. Verifica colisão do raio de coleta com os obstáculos
            in_collision = any(obs.check_collision_goal(candidate_pos,self.MIN_REACH) for obs in self.obstacles)
            
            if in_collision:
                continue
            
            # 3. Verifica sobreposição geométrica com outros Goals (distância <= 2x raio)
            # Obs: Se self.goals estiver vazia, o 'any' já retorna False automaticamente.
            is_overlapping = any(np.linalg.norm(g.pos - candidate_pos) <= self.MIN_REACH * 2 for g in self.goals)
            
            if not is_overlapping:
                reward = rd.uniform(10, 300)
                
                print(f"- [{(candidate_pos[0]):.4f}, {(candidate_pos[1]):.4f}, {reward:.4f}]")
                n = n+1

    def _print_goals_vns_prm_pop_format(self, cenario):
        print(f"0 {round(self.start_pos[0])} {round(self.start_pos[1])} 0")
        print(f"1 {round(self.start_pos[0])} {round(self.start_pos[1])} 0")
        k = 2
        total_reward = 0
        for g in self.goals:
            print(f"{k} {g.pos[0]} {g.pos[1]} {g.reward}")
            k = k + 1
            total_reward = g.reward + total_reward
        print(f"Cenario: {cenario}")
        print(f"Recompensa total do cenario: {round(total_reward)}")
        print(f"Numero de goals: {k - 2}")

        


# Funcao para gerar goals, mexer depois se precisar

# rd.seed(42) #Mude o valor da seed caso queira valores aleatorios iguais.
        
        # Este while e um resquicio de quando criavamos os goals de forma aleatoria, use apenas para criar novos cojuntos de goal para testes!
        # while len(self.goals) < 50:
        #     candidate_pos = np.array([float(rd.randn(25, 975)), float(rd.randn(25, 975))])
        
        # Evitar que goals se sobrepoem (raio) distancia entre eles >= raio de coleta

        #     # Check if candidate position is inside a black obstacle pixel
        #     if not self.mapa.collision(candidate_pos):
        #         reward = rd.randn(10, 300)
        #         print(f"[{int(candidate_pos[0])}, {int(candidate_pos[1])}, {reward}],")
        #         self.goals.append(Goal(candidate_pos, reward, 420))