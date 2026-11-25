import numpy as np

class CostMap:
    def __init__(self, base_cost=1.0):
        self.c0 = base_cost
        # Lista de obstáculos no formato: (x, y, Amplitude, sigma)
        self.obstacles = []
        
    def add_gaussian_cost(self, x, y, A, sigma):
        self.obstacles.append((x, y, A, sigma))

    def get_cost_at(self, x, y):
        """
        Avalia a função C(x,y) em um ponto específico.
        """
        cost = self.c0
        for (ox, oy, A, sigma) in self.obstacles:
            # Equação da Gaussiana 2D
            cost += A * np.exp(-((x - ox)**2 + (y - oy)**2) / (2 * sigma**2))
        return cost

    def travel_cost(self, p1, p2, num_steps=500):
        """
        Calcula o custo da integral de linha de p1 até p2.
        p1, p2: listas ou arrays [x, y]
        num_steps: em quantos pedaços dividir a reta
        """
        p1 = np.array(p1)
        p2 = np.array(p2)
        
        # Distância Euclidiana real entre os pontos
        dist = np.linalg.norm(p2 - p1)
        if dist == 0:
            return 0.0

        # Tamanho de cada pequeno passo na discretização (ds)
        ds = dist / num_steps

        # Gera 'num_steps' pontos ao longo da reta (excluindo o p1 puro e indo até p2)
        # Usamos interpolação linear paramétrica: P(t) = P1 + t * (P2 - P1)
        t_values = np.linspace(0, 1, num_steps)
        
        total_cost = 0.0
        for t in t_values:
            # Coordenada (x,y) do ponto intermediário atual
            pt = p1 + t * (p2 - p1)
            
            # Avalia C(x,y) nesse ponto e multiplica pelo tamanho do passo ds
            total_cost += self.get_cost_at(pt[0], pt[1]) * ds
            
        return total_cost

# ==========================================
# Exemplo de Uso
# ==========================================
# cost_map = CostMap(base_cost=1.0)

# # Adicionando um obstáculo no centro (500, 500) com amplitude 10 e raio de influência 50
# cost_map.add_gaussian_cost(x=500, y=500, A=10, sigma=50)

# # Ponto 1 e Ponto 2
# p_inicio = [500, 400]
# p_fim    = [500, 500] # Passa rasgando bem no centro da gaussiana
# n=0

# print(f"Distância real: {np.linalg.norm(np.array(p_fim) - np.array(p_inicio)):.2f}")

# while (n<500):
#     custo = cost_map.travel_cost(p_inicio, p_fim, num_steps=n)
#     print(f"Custo mapeado para n={n} : {custo:.2f}")
#     n+=1

# # num_step recomendado: 200