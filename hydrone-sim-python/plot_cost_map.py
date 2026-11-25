import matplotlib.pyplot as plt
import numpy as np
from cost_map import CostMap
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

def plot_cost_map_3d_with_obstacles(
    cost_map,
    bounds=(-5, 5, -5, 5),
    resolution=200,
    poly_obstacles=None,
    poly_height=0.5,
):
  """Plota o mapa de custo 3D combinando o campo potencial (gaussianas) e obstáculos poligonais (prismas 3D).

  Parameters:
  - cost_map: Instância da classe CostMap
  - bounds: Tupla (xmin, xmax, ymin, ymax) com os limites do gráfico
  - resolution: Número de pontos da grade de amostragem
  - poly_obstacles: Lista de polígonos, onde cada polígono é uma lista de
  vértices [[x1, y1], [x2, y2], ...]
  - poly_height: Altura (z) dos obstáculos geométricos (ex: 0.5m)
  """
  xmin, xmax, ymin, ymax = bounds

  # 1. Geração da grade da superfície de custo (Gaussianas)
  x = np.linspace(xmin, xmax, resolution)
  y = np.linspace(ymin, ymax, resolution)
  X, Y = np.meshgrid(x, y)

  Z = np.full(X.shape, cost_map.c0, dtype=float)
  for ox, oy, A, sigma in cost_map.obstacles:
    Z += A * np.exp(-((X - ox) ** 2 + (Y - oy) ** 2) / (2 * sigma**2))

  # 2. Configuração do Plot 3D

  fig = plt.figure(figsize=(12, 8), dpi=100)
  ax = fig.add_subplot(111, projection='3d')

  # Plota a superfície contínua (Costmap)
  surf = ax.plot_surface(
      X, Y, Z, cmap='YlOrRd', edgecolor='none', alpha=0.75, antialiased=True)

  # 3. Renderização dos Obstáculos Poligonais 3D
  if poly_obstacles:
    for poly in poly_obstacles:
      poly = np.array(poly)
      n_verts = len(poly)

      z_base = cost_map.c0  # Apoia o bloco sobre o custo base
      z_top = z_base + poly_height

      faces = []

      # Face inferior (base)
      bot_face = [[p[0], p[1], z_base] for p in poly]
      faces.append(bot_face)

      # Face superior (topo)
      top_face = [[p[0], p[1], z_top] for p in poly]
      faces.append(top_face)

      # Faces laterais (conecta vertice i com i+1)
      for i in range(n_verts):
        next_i = (i + 1) % n_verts
        side_face = [
            [poly[i][0], poly[i][1], z_base],
            [poly[next_i][0], poly[next_i][1], z_base],
            [poly[next_i][0], poly[next_i][1], z_top],
            [poly[i][0], poly[i][1], z_top],
        ]
        faces.append(side_face)

      # Desenha o objeto 3D
      prism = Poly3DCollection(
          faces,
          facecolors='darkred',
          edgecolors='black',
          linewidths=1.0,
          alpha=0.85,
      )
      ax.add_collection3d(prism)

  # Configurações visuais
  ax.set_xlabel('X (m)', labelpad=10)
  ax.set_ylabel('Y (m)', labelpad=10)
  ax.set_zlabel('Custo Z', labelpad=10)
  ax.set_title('Mapa 3D: Campo Potencial + Obstáculos Geométricos')
  ax.set_xlim(xmin, xmax)
  ax.set_ylim(ymin, ymax)

  cbar = fig.colorbar(surf, ax=ax, shrink=0.55, aspect=12, pad=0.1)
  cbar.set_label('Intensidade do Custo')

  ax.view_init(elev=35, azim=-45)
  plt.tight_layout()
  plt.show()


# ==========================================
# Exemplo de Execução
# ==========================================
if __name__ == '__main__':
  # 1. Instancia o CostMap com base
  cost_map = CostMap(base_cost=1.0)

  # Adiciona um campo de custo gaussiano próximo aos obstáculos
  cost_map.add_gaussian_cost(-6,-1.32,10,1)
  cost_map.add_gaussian_cost(3.375,2.125,5,1)

  # 2. Definição dos obstáculos poligonais (vértices [x,y])
  obs_geometricos = [
      # Obstáculo 1
      [[-4.65, 0.55], [-2.35, 0.55], [-2.35, 3.85], [-4.65, 3.85]],
      # Obstáculo 2
      [[2.4, 2.55], [4.7, 2.55], [4.7, 4.85], [2.4, 4.85]],
      # Obstáculo 3
      [[-4.25, -4.05], [0.05, -4.05], [0.05, -2.75], [-4.25, -2.75]],
  ]

  # 3. Plota o gráfico combinando o campo contínuo com os blocos com altura = 0.5m
  plot_cost_map_3d_with_obstacles(
      cost_map,
      bounds=(-7.5,7.5,-7.5,7.5),
      resolution=200,
      poly_obstacles=obs_geometricos,
      poly_height=0.5,
  )