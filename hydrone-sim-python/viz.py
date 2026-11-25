import matplotlib
import cv2
#matplotlib.use('Qt5Agg')   # Qt5 ja esta instalado via PyQt5
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.patches import Polygon as MplPolygon
from matplotlib.cm import ScalarMappable
from matplotlib.colors import Normalize
import numpy as np

'''
Comente o comando abaixo para a janela de plot poder aparecer na tela
'''
# matplotlib.use('Agg')


class Visualization:
    """
    Classe de visualizacao baseada em matplotlib.
    Mantem o mesmo contrato de metodos da versao PyQtGraph:
        __init__(), draw_map(mapa), update(rrt), show(), resize(w, h), save(filepath)
    """

    def __init__(self, bounds):
        plt.ion()   # modo interativo: permite atualizar sem bloquear
        self.bounds = bounds
        self.fig = plt.figure(figsize=(9, 8))

        # GridSpec: plot principal + colorbar fino
        gs = self.fig.add_gridspec(1, 2, width_ratios=[20, 1], wspace=0.08)
        self.ax  = self.fig.add_subplot(gs[0])
        self.ax.set_aspect('equal')
        self.cax = self.fig.add_subplot(gs[1])


        # Colormap de azul ate roxo, range 10 a 300
        self.cmap = plt.cm.cool
        self.norm = Normalize(vmin=10, vmax=300)

        # ScalarMappable liga o colormap ao colorbar
        sm = ScalarMappable(cmap=self.cmap, norm=self.norm)
        sm.set_array([])

        self.colorbar = self.fig.colorbar(sm, cax=self.cax)
        self.colorbar.set_label('Reward', fontsize=13, color='black')
        self.colorbar.ax.yaxis.set_tick_params(color='black', labelsize=11)
        plt.setp(self.colorbar.ax.yaxis.get_ticklabels(), color='black')
        for spine in self.colorbar.ax.spines.values():
            spine.set_edgecolor('black')
            spine.set_linewidth(1.5)

        self.cost_grid = None
        self.map_img = None

        self._style_axes()

    def draw_map(self, mapa):
        """Carrega o mapa para ser desenhado no fundo.

        Pode receber:
        - Uma string com o caminho da imagem (ex: 'imgs/mapa.png')
        - Uma matriz numpy (imagem em escala de cinza ou RGB)
        - Um objeto da sua classe Map (acessando mapa.mapa)
        """
        if isinstance(mapa, str):
        # Carrega via OpenCV e converte para RGB (ou escala de cinza)
            img = cv2.imread(mapa)
        if img is not None:
            self.map_img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        elif hasattr(mapa, 'mapa'):
            self.map_img = mapa.mapa
        elif isinstance(mapa, np.ndarray):
            self.map_img = mapa

    def _precompute_costmap(self, rrt):
        # Cria um grid com resolução menor (150x100) para processamento rápido
        # Você pode aumentar para 1500x1000 se quiser qualidade máxima (mas demora mais a primeira carga)
        x = np.linspace(self.bounds[0], self.bounds[1], 1000)
        y = np.linspace(self.bounds[2], self.bounds[3], 1000)
        X, Y = np.meshgrid(x, y)
        
        # Inicializa o mapa com o custo base
        Z = np.full(X.shape, rrt.cost_map.c0)
        
        # Soma as funções Gaussianas de cada obstáculo (Assumindo a estrutura que discutimos antes)
        for (ox, oy, A, sigma) in rrt.cost_map.obstacles:
            Z += A * np.exp(-((X - ox)**2 + (Y - oy)**2) / (2 * sigma**2))
            
        self.cost_grid = Z

    # ── Estilizacao ───────────────────────────────────────────────────

    def _style_axes(self):
        """Aplica toda a estilizacao dos eixos (chamada apos cada cla())."""
        self.ax.set_xlim(self.bounds[0], self.bounds[1])
        self.ax.set_ylim(self.bounds[2], self.bounds[3])


        self.ax.set_xlabel('x [m]', fontsize=14, color='black')
        self.ax.set_ylabel('y [m]', fontsize=14, color='black')

        #self.ax.axhspan(0, 500, color='lightskyblue', alpha=0.4, zorder=0)

        self.ax.set_facecolor('white')
        self.fig.patch.set_facecolor('white')

        # Bordas pretas em todos os lados (fecha a caixa)
        for spine in self.ax.spines.values():
            spine.set_edgecolor('black')
            spine.set_linewidth(2)

        # Ticks pretos
        self.ax.tick_params(axis='both', colors='black', labelsize=13, width=2)

    # ── API publica ───────────────────────────────────────────────────

    def show(self):
        """Exibe a janela sem bloquear a execucao."""
        plt.show(block=False)
        self.fig.canvas.flush_events()

    def close(self):
        """Fecha todas as janelas abertas"""
        plt.close('all')
        self.fig.canvas.flush_events()

    def resize(self, width: int, height: int):
        """Redimensiona a janela (equivalente a plot.resize() do Qt)."""
        dpi = self.fig.get_dpi()
        self.fig.set_size_inches(width / dpi, height / dpi)

    def save(self, filepath: str):
        """Salva a figura atual em arquivo (substitui plot.grab() + pixmap.save())."""
        self.fig.savefig(
            filepath,
            bbox_inches='tight',
            dpi=150,
            facecolor='white',
            edgecolor='none'
        )

    def update(self, rrt):
        """
        Limpa e redesenha tudo.
        Chamada a cada iteracao da simulacao ou ao final para salvar.
        """
        # Limpa tudo — evita a repeticao de items que ocorria no pyqtgraph
        self.ax.cla()
        self._style_axes()

        # Informações de título
        self.fig.suptitle(rrt.SCENARIO + "/Budget: " + str(rrt.BUDGET) + "/Iter: " + str(rrt.MAX_ITER) + "/Dist: " + str(rrt.DIST), y=0.95, fontsize='xx-large', 
            fontweight='bold')
        
        if self.map_img is not None:
        # extent mapeia os pixels da imagem para [xmin, xmax, ymin, ymax]
        # origin='upper' é o padrão de imagens (linha 0 no topo).
        # Se a sua imagem ficar de ponta cabeça, mude para origin='lower' ou use cv2.flip(img, 0).
            self.ax.imshow(
                self.map_img,
                extent=[
                    self.bounds[0],
                    self.bounds[1],
                    self.bounds[2],
                    self.bounds[3],
                ],
                origin='upper',
                cmap='gray',
                alpha=0.6,  
                zorder=0,
            )

        # ── NOVO: Plot do Heatmap no Fundo (zorder=1) ─────────────────
        # Verifica se o objeto rrt tem o cost_map instanciado
        if hasattr(rrt, 'cost_map') and rrt.cost_map is not None:
            # Calcula o grid na primeira iteração e salva no self.cost_grid
            if self.cost_grid is None:
                self._precompute_costmap(rrt)
            
            # Plota a imagem no fundo. 
            # cmap='YlOrRd' (Amarelo, Laranja e Vermelho) dá um contraste ótimo para mapas de risco
            self.ax.imshow(
                self.cost_grid, 
                extent=[
                    self.bounds[0],
                    self.bounds[1],
                    self.bounds[2],
                    self.bounds[3],
                ], 
                origin='lower', 
                cmap='YlOrRd', 
                alpha=0.5,       # Transparência para não ofuscar o grid/árvore
                zorder=1
            )

        # ── Arestas e vertices das arvores ────────────────────────────
        for t in rrt.trees:
            for e in t.edges:
                self.ax.plot(
                    [e[0][0], e[1][0]],
                    [e[0][1], e[1][1]],
                    color='black', linewidth=1.5, zorder=5
                )

            if t.vertexes:
                vx = [v[0] for v in t.vertexes]
                vy = [v[1] for v in t.vertexes]
                self.ax.plot(
                    vx, vy,
                    'o', color='orange',
                    markersize=3, linewidth=0, zorder=5
                )

        for en in rrt.end_nodes:
            self.ax.plot(
                    en[0], en[1],
                    'o', color='orange',
                    markersize=3, linewidth=0, zorder=5
                )

        # ── Obstaculos ────────────────────────────────────────────────
        for obs in rrt.obstacles:
            poly = MplPolygon(
                obs.polygon_points, closed=True,
                facecolor='red', edgecolor='black', zorder=3
            )
            self.ax.add_patch(poly)

        # ── Goals ────────────────────────────────────────────
        for goal in rrt.goals:
            # Borda indica o estado do goal
            if goal in rrt.reached_goals:
                border_color = 'lime'
            elif goal in rrt.unfeasible_goals:
                border_color = 'red'
            else:
                border_color = 'grey'

            # Preenchimento vem do colormap baseado no reward
            fill_color = self.cmap(self.norm(goal.reward))

            circle = mpatches.Circle(
                (float(goal.pos[0]), float(goal.pos[1])),
                radius=rrt.MIN_REACH,
                linewidth=1,
                edgecolor=border_color,
                facecolor=fill_color,
                zorder=4
            )
            self.ax.add_patch(circle)
            

        # ── Start ─────────────────────────────────────────────────────

        start_circle = mpatches.Circle(
            (float(rrt.start_pos[0]), float(rrt.start_pos[1])),
                            radius=rrt.MIN_REACH,
                            linewidth=1,
                            edgecolor='grey',
                            facecolor='none',
                            zorder=4
                        )
        self.ax.add_patch(start_circle)

        if (rrt.start_pos != rrt.ending_pos).any():
            side = rrt.MIN_REACH*2
            x = float(rrt.ending_pos[0]) - side/2
            y = float(rrt.ending_pos[1]) - side/2
            ending_rectangle = mpatches.Rectangle(
                        (x,y),
                        width = side,
                        height = side,
                        linewidth=1,
                        edgecolor='grey',
                        facecolor='none',
                        zorder=4
                        )
            self.ax.add_patch(ending_rectangle)

        # self.ax.text(
        #     float(rrt.start_pos[0]), float(rrt.start_pos[1]),
        #     'START',
        #     ha='center', va='center',
        #     fontsize=9, color='black', zorder=5
        # )

        # Atualiza a janela

        self.fig.canvas.draw()
        self.fig.canvas.flush_events()