# Projeto VINI — Robô Móvel Autônomo (Projeto Final)

Repositório do projeto final da disciplina contendo a modelagem mecânica 3D (SolidWorks), as cenas de simulação (CoppeliaSim), o pipeline de navegação autônoma (Mapeamento, Localização MCL, Planejamento RRT e Controle Cinemático) e a teleoperação via manete USB do triciclo **V.I.N.I.**.

![Pipeline da Arquitetura](mapeamento/pipeline_arquitetura.png)

---

## Estrutura do Projeto

```text
Projeto_VINI/
├── projeto_final_vini.ipynb        # Notebook principal do Projeto Final (Módulos 1 a 4)
├── mapeamento/                     # Módulo 1: Grades de Ocupação 2D e Calibração
│   ├── mapa_cidade.jpg             # Vista superior original do cenário urbano
│   ├── grid_rrt.npy                # Camada de Navegação (0=Asfalto Livre C-Space, 1=Ocupado)
│   ├── grid_mcl.npy                # Camada de Obstáculos Físicos 3D p/ LiDAR (0=Livre, 1=Prédios)
│   ├── map_config.json             # Metadados de calibração (resolution, origin_x, origin_y, width, height)
│   ├── visualizacao_mapas.png      # Comparativo visual das camadas geradas
│   └── pipeline_arquitetura.png    # Diagrama de blocos da arquitetura do sistema
├── cad_solidworks/                 # Peças (.SLDPRT) e montagem completa (.SLDASM) no SolidWorks
│   ├── Montagem_VINI.SLDASM        # Montagem principal do robô VINI
│   └── *.SLDPRT                    # Peças do chassi, direção, rodas, hubs e servo MG996R
├── simulacao_coppeliasim/          # Cenas de simulação do CoppeliaSim (.ttt)
│   ├── Cena_Cidade-VINI.ttt        # Cena urbana completa com o robô VINI
│   └── Novo_modelo_Cena.ttt        # Cena do modelo atualizado do VINI
├── controle/                       # Teleoperação via manete USB (Xbox / Gamepad) via ZMQ Remote API
│   ├── controle_manete_vini.ipynb  # Jupyter Notebook interativo de teleoperação
│   └── controle_manete_vini.py     # Script Python standalone para execução direta no terminal
├── requirements.txt                # Dependências Python do projeto
└── .gitignore                      # Ignora arquivos temporários do SolidWorks (~$*), Jupyter e Python
```

---

## Módulos do Sistema Autônomo ([projeto_final_vini.ipynb](projeto_final_vini.ipynb))

1. **Módulo 1 — Mapeamento (`mapeamento/`)**:
   - Processa a geometria vista de cima do cenário urbano do CoppeliaSim (`mapa_cidade.jpg`) gerando duas grades binárias alinhadas (`1024 x 976`):
     - `grid_rrt.npy`: Camada de navegação com asfalto contínuo (`cv2.MORPH_CLOSE`) e inflação C-Space para manter distância segura do meio-fio.
     - `grid_mcl.npy`: Camada física contendo exclusivamente os **20 prédios 3D** que refletem o laser do `fastHokuyo`.
     - `map_config.json`: Parâmetros de transformação mundo real $(x, y) \leftrightarrow (col, row)$.
2. **Módulo 2 — Localização Probabilística (MCL / Filtro de Partículas)** *(em desenvolvimento)*
3. **Módulo 3 — Planejamento de Caminhos (RRT com restrição cinemática)** *(em desenvolvimento)*
4. **Módulo 4 — Controle Cinemático (Modelo Bicicleta / Ackermann)** *(em desenvolvimento)*
