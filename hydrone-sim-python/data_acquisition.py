from Reward_Guided_RRT import Reward_Guided_RRT
from viz import Visualization
from Node import Node

import os
import sys
import csv
import yaml
import shutil

"""
Code made to run multiple simulations with different parameters to analytical data analysis.
The data will be stored at simulation_data diretory, with runs separated by "Number of iteration"_"step distance"_"Scnery difficulty"
"""

def main(configs_n: int, repeats: int,  configs: list):
    sim_index = 0

    # Deleta os resultados salvos antes de rodar o código
    os.makedirs("simulation_data/", exist_ok=True) 
    shutil.rmtree("simulation_data")
    
    for configs_n in range(configs_n):
        rp = 1
        config_inst = configs[configs_n]

        os.makedirs(
            f"simulation_data/{config_inst[0]}_{config_inst[1]}_{config_inst[2]}/final_budget_reward",
            exist_ok=True
        )
        results_file = f"simulation_data/{config_inst[0]}_{config_inst[1]}_{config_inst[2]}/final_budget_reward/results_rrt.csv"

        with open(results_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(["Repeticao", "Budget_Usado", "Reward_Coletado", "Budget_Usado%", "Reward_Coletado%"])

        while rp <= repeats:

            rrt = Reward_Guided_RRT(config_inst[0], config_inst[1], config_inst[2])
            
            """
            Funcoes auxiliares para gerar configuracoes de cenarios ou formatar essas informacoes
            """
            # rrt._genarate_randon_goals(40)
            # rrt._print_goals_vns_prm_pop_format(config_inst[2])

            k_atual = 0

            while k_atual < rrt.MAX_ITER and not rrt.done:
                rrt.step()
                k_atual += 1

            # Para poder considerar os goals pegos no retorno voce precisa chamar o print_solution(), ja que a extensao da lista fica nessa funcao
            # Concerteza existe uma solucao melhor, mas vai essa por enquanto
            rrt.print_solution()

            with open(results_file, 'a', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([
                    rp,
                    f"{rrt.total_budget_used}",
                    f"{rrt.total_reward_collected}",
                    f"{rrt.total_budget_used / rrt.BUDGET}",
                    f"{rrt.total_reward_collected / rrt.total_reward_available}"
                ])

            # Salva todos os goals usados na bateria de testes
            os.makedirs(f"simulation_data/{config_inst[0]}_{config_inst[1]}_{config_inst[2]}/all_goals", exist_ok=True)
            if rp < 2:
                all_goals_file = f"simulation_data/{config_inst[0]}_{config_inst[1]}_{config_inst[2]}/all_goals/all_goals_{rp}.csv"

            with open(all_goals_file, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(["Indice", "X", "Y", "Reward"])
                for i, g in enumerate(rrt.goals, start=1):
                    writer.writerow([g.index, int(g.pos[0]), int(g.pos[1]), g.reward])

            print(f"[INFO] Goals salvos em {all_goals_file}", file=sys.stderr)

            # Salva apenas os goals visitados na bateria de testes
            os.makedirs(f"simulation_data/{config_inst[0]}_{config_inst[1]}_{config_inst[2]}/visited_goals", exist_ok=True)

            visited_goals_file = f"simulation_data/{config_inst[0]}_{config_inst[1]}_{config_inst[2]}/visited_goals/visited_goals_{rp}.csv"

            with open(visited_goals_file, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(["Indice", "X", "Y", "Reward"])
                for i, g in enumerate(rrt.reached_goals, start=1):
                    writer.writerow([g.index, int(g.pos[0]), int(g.pos[1]), g.reward])

            print(f"[INFO] Goals salvos em {visited_goals_file}", file=sys.stderr)

            # Salva os nodes da mescla de todas as arvores de ida e a ultima de volta
            os.makedirs(f"simulation_data/{config_inst[0]}_{config_inst[1]}_{config_inst[2]}/tree_nodes", exist_ok=True)
            outward_node_file = f"simulation_data/{config_inst[0]}_{config_inst[1]}_{config_inst[2]}/tree_nodes/tree_nodes_{rp}.csv"

            with open(outward_node_file, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(["Id", "X", "Y", "Reward_Coletado", "Budget_Usado"])
                if rrt.all_outward_tree is not None:
                    for node in rrt.all_outward_tree.nodes.values():
                        writer.writerow([
                            node.id,
                            float(node.position[0]),
                            float(node.position[1]),
                            float(node.reward_collected),
                            float(node.budget_used)
                        ])

            print(f"[INFO] Nodes salvos em {outward_node_file}", file=sys.stderr)

            # Caminho completo encontrado pelo algoritimo, comecando e terminando pelo start
            os.makedirs(f"simulation_data/{config_inst[0]}_{config_inst[1]}_{config_inst[2]}/full_path_nodes", exist_ok=True)

            full_path_file = f"simulation_data/{config_inst[0]}_{config_inst[1]}_{config_inst[2]}/full_path_nodes/full_path_nodes_{rp}.csv"

            # Monta o caminho completo: going_path (todas as pernas) + returning_path
            full_path: list[Node] = []

            # Concatena todas as pernas do going_path
            # Pula o primeiro node de cada perna após a primeira (junction duplicado)
            for i, seg in enumerate(rrt.going_path):
                full_path.extend(seg if i == 0 else seg[1:])

            # Concatena returning_path
            # Pula o primeiro node (mesmo que o último do going_path)
            if rrt.returning_path:
                full_path.extend(rrt.returning_path[1:])

            with open(full_path_file, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(["Id", "X", "Y", "Reward_Coletado", "Budget_Usado"])

                # Primeiro node: START
                writer.writerow([
                    "START",
                    float(rrt.start_pos[0]),
                    float(rrt.start_pos[1]),
                    0.0,
                    0.0
                ])

                # Nodes do caminho
                for node in full_path:
                    writer.writerow([
                        node.id,
                        float(node.position[0]),
                        float(node.position[1]),
                        float(node.reward_collected),
                        float(node.budget_used)
                    ])

                # Último node: START novamente
                writer.writerow([
                    "START",
                    float(rrt.start_pos[0]),
                    float(rrt.start_pos[1]),
                    float(full_path[-1].reward_collected) if full_path else 0.0,
                    float(full_path[-1].budget_used)      if full_path else 0.0
                ])

            print(f"[INFO] Caminho completo salvo em {full_path_file}", file=sys.stderr)

            # ── Abre a janela só no final ────────────────────────────────
            plot = Visualization()

            rrt.show_final_path()
            plot.update(rrt)

            # ── Salva screenshot full HD e fecha ─────────────────────────
            os.makedirs(f"simulation_data/{config_inst[0]}_{config_inst[1]}_{config_inst[2]}/path_print", exist_ok=True)

            plot.save(f"simulation_data/{config_inst[0]}_{config_inst[1]}_{config_inst[2]}/path_print/path_print_{rp}.pdf")
            plot.close()

            print(f"[INFO] Screenshot salvo: path_{rp}.pdf", file=sys.stderr)

            # ── Visualização 2: Árvore completa ──────────────────────────
            plot_full = Visualization()


            # Atualiza _viz_all_out com os dados da árvore acumulada
            if rrt.all_outward_tree is not None:
                rrt._viz_all_out.edges    = [list(e) for e in rrt.all_outward_tree.all_edges]
                rrt._viz_all_out.vertexes = rrt.all_outward_tree.all_positions

            # Substitui trees temporariamente para exibir só a árvore acumulada
            rrt.trees = [rrt._viz_all_out]
            plot_full.update(rrt)

            os.makedirs(f"simulation_data/{config_inst[0]}_{config_inst[1]}_{config_inst[2]}/full_rrt_print", exist_ok=True)

            plot_full.save(f"simulation_data/{config_inst[0]}_{config_inst[1]}_{config_inst[2]}/full_rrt_print/full_rrt_print_{rp}.pdf")
            plot_full.close()
            print(f"[INFO] Full RRT salvo: full_rrt_print_{rp}.pdf", file=sys.stderr)

            sim_index += 1
            rp += 1

"""
Change the parameters of your simulations by adding the configurations you want:

rep       -> number of repetions of a set of parameters
it        -> number of tree iterations
dists     -> step distance of tree nodes
scneraios -> scenerys configuration, must been previously buit at scenerys.yaml
"""


# Configurações dos testes a serem feitos
rep = 15                                  # Número de vezes que cada configuração de teste deve ser rodada
it = [10000]                 # Número de iterações
dists = [50]                     # Distância de expansão da árvore
scenarios = ["CM"]    # Cenários
configs = []

for i in range(len(it)):
    for j in range(len(dists)):
        for k in range(len(scenarios)):
            configs.append([it[i], dists[j], scenarios[k]])

main(len(configs), rep, configs)

sys.exit(0)