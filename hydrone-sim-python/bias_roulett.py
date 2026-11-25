import numpy as np
from cost_map import CostMap
from goal import Goal

class Roulette:
    def __init__(self, real_goals: list[Goal], cost_map: CostMap, bounds):
        self.real_goals = real_goals
        self.cost_map = cost_map
        self.bounds = bounds

        self.prob_random = 0.3
        self.reward_bias = 1.0
        self.distance_bias = 1.0
        self.empty = False
        
        # Intensidade do filtro de baixo custo (1.0 = normal, 2.0 = rejeição mais severa)
        self.cost_bias_power = 1.0 

    def _sample_low_cost_pos(self, max_attempts=30) -> np.ndarray:
        """
        Gera uma posição aleatória com maior probabilidade de estar em regiões de baixo custo.
        """
        c0 = self.cost_map.c0

        for _ in range(max_attempts):
            rx = np.random.uniform(self.bounds[0], self.bounds[1])
            ry = np.random.uniform(self.bounds[2], self.bounds[3])
            
            cost = self.cost_map.get_cost_at(rx, ry)
            
            # Probabilidade inversamente proporcional ao custo
            p_accept = (c0 / cost) ** self.cost_bias_power
            
            # Aceita o ponto se o número aleatório for menor que a probabilidade
            if np.random.random() < p_accept:
                return np.array([rx, ry])

        # Se ultrapassar as tentativas (em mapas extremamente poluídos), 
        # retorna o último sorteado como fallback para não travar o loop
        return np.array([rx, ry])

    def spin(self, reached_goals: list, root_pos: np.ndarray):
        """
        Sorteia um objetivo ignorando aqueles que já estão conectados à árvore.
        Retorna: (posicao_np_array, is_random_flag)
        """
        available_goals = [
            g for g in self.real_goals 
            if tuple(g.pos) not in reached_goals
        ]

        if not available_goals:
            self.empty = True
            # Se não há mais goals disponíveis, sorteia um ponto aleatório com viés de baixo custo
            return None
        else:
            self.empty = False
        
        # Sorteio com probabilidade 'prob_random' de ir para amostragem aleatória
        if np.random.random() < self.prob_random:
            return self._sample_low_cost_pos()
        
        # Seleção de Goal por Roleta (Incentivado por Recompensa / Distância)
        weights = []
        for g in available_goals:
            dist = self.cost_map.travel_cost(root_pos, g.pos)
            dist = max(dist, 1.0)
            w = (g.reward * self.reward_bias) / (dist * self.distance_bias)
            weights.append(w)
            
        total_weight = sum(weights)
        probs = [w / total_weight for w in weights]
        
        selected = np.random.choice(available_goals, p=probs)
        return np.array(selected.pos)