from Reward_Guided_RRT import Reward_Guided_RRT
from viz import Visualization

import os
import sys
import yaml
import time

#==================================================================
# Main
#==================================================================

this_dir  = os.path.dirname(os.path.abspath(__file__))
yaml_path = os.path.join(this_dir, 'scenerys.yaml')

scenario   = sys.argv[1]       if len(sys.argv) > 1 else "LOW"
iterations = int(sys.argv[2])  if len(sys.argv) > 2 else 1500
step       = float(sys.argv[3]) if len(sys.argv) > 3 else 50


VALID_SCENARIOS = None  # sera carregado do yaml

def load_valid_scenarios(yaml_path: str) -> list:
    if not os.path.exists(yaml_path):
        print(f"[ERRO] Scenery path file not found: {yaml_path}")
        sys.exit(1)
    with open(yaml_path, 'r') as f:
        config = yaml.safe_load(f)
    return list(config.keys())

def parse_args(yaml_path: str):
    valid_scenarios = load_valid_scenarios(yaml_path)

    # ── scenario ──────────────────────────────────────────────────────
    scenario = sys.argv[1] if len(sys.argv) > 1 else "LOW"

    if scenario not in valid_scenarios:
        print(f"[ERRO] Scnario '{scenario}' invalid.")
        print(f"       Scenarios available: {valid_scenarios}")
        sys.exit(1)

    # ── iterations ────────────────────────────────────────────────────
    if len(sys.argv) > 2:
        try:
            iterations = int(sys.argv[2])
        except ValueError:
            print(f"[ERRO] 'iterations' must be a int. Received: '{sys.argv[2]}'")
            sys.exit(1)

        if iterations <= 0:
            print(f"[ERRO] 'iterations' must be bigger then zero. Received: {iterations}")
            sys.exit(1)
    else:
        iterations = 1500

    # ── step ──────────────────────────────────────────────────────────
    if len(sys.argv) > 3:
        try:
            step = sys.argv[3]
        except ValueError:
            print(f"[ERRO] 'step' must be a int. Received: '{sys.argv[3]}'")
            sys.exit(1)

        if step <= 0:
            print(f"[ERRO] 'step'  must be bigger then zero. Received: {step}")
            sys.exit(1)
    else:
        step = 50

    print(f"[INFO] Configuration: scenario={scenario} | iterations={iterations} | step={step}")
    return scenario, iterations, step


rrt = Reward_Guided_RRT(iterations, step, scenario)

plot = Visualization(rrt.BOUNDS)
plot.draw_map('maps/city_map.jpeg')
plot.show()

#rrt._genarate_randon_goals()

k_atual = 0

start = time.perf_counter()

while k_atual < rrt.MAX_ITER and not rrt.done:
    rrt.step()
    k_atual += 1

    if (k_atual % 100) == 0:
        plot.update(rrt)

end = time.perf_counter()
time = end - start

rrt.print_solution()
rrt.show_final_path()
plot.update(rrt)
print("***Simulation concluded***")
print(f"Simulation time: {time:.2f} seconds")

# print("Initializing path improvement")
# rrt._improve_path()

# rrt.print_solution()
# rrt.show_final_path()

# # Mostra e atualiza a segunda janela com o caminho otimizado
# plot2.show()
# plot2.update(rrt)

# print("***Final optimization concluded***")
# print(f"Simulation time: {time:.2f} seconds")

# # Agora sim, colocamos o input e o fechamento no final para as DUAS ficarem abertas
input("Press enter to close both windows...")
plot.close()

sys.exit()