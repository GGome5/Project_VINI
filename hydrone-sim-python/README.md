# Reward-Guided RRT*

Given a set of reward-bearing locations in an environment with physical obstacles and a limited budget used to navigate to those locations, the navigation and visitation order of reward-bearing locations in that environment within a budget is called the Physical Orienteering Problem (POP).

This repository's algorithm shows an approach to attempt to solve this problem by using sequential RRT* that expands toward promising locations by using a "roulette" method.

A more complete discussion can be found in this method's paper below:

"paper url if published in ICRA"
 
<!-- 
    Project description goes here.
    Describe the problem you are solving, the motivation behind this work,
    and the key contributions of this approach.
-->

---

## Organization

VeRLab | Laboratory of Computer Vision & Robotics – Computer Science Department of Universidade Federal de Minas Gerais, Brazil

## Authors

Armando Alves Neto — Project Surpevisor and Professor at the Department of Electronic Engineering (DELT/UFMG)
Douglas Guimarães Macharet — Project Surpevisor and Professor at the Department of Computer Science (DCC/UFMG)

Gabriel Ferreira — Scientific iniation member and undergraduate student of Eletrical Engineering (UFMG)
Vinicius Araujo de Oliveira — Scientific iniation member and undergraduate student of Control and Automation Engineering (UFMG)
<!-- 
- Author Name — role or affiliation
- Author Name — role or affiliation
-->

---

## Dependencies

This project requires Python 3.10 or higher. Install all dependencies with:

```bash
pip install numpy matplotlib pyyaml shapely
```

---

## Project Structure

```
.
├── Reward_Guided_RRT.py    # Core RRT* planning algorithm
├── run_simulation.py       # Interactive single-run simulation with visualization
├── data_acquisition.py     # Batch simulation runner for data collection
├── viz.py                  # Visualization class (matplotlib-based)
├── scenerys.yaml           # Scenario definitions (goals and obstacles)
├── Node.py                 # Node data structure
├── pandas_analytic.py      # Data analyzes of simulation_data/
└── simulation_data/        # Output directory (created automatically)
```

---

## Usage

### `run_simulation.py` — Interactive Simulation

Runs a single simulation with live visualization. The window updates during
tree expansion and displays the final path at the end.

```bash
python3 run_simulation.py <scenario> <iterations> <step>
```

| Argument     | Type   | Default | Description                                      |
|--------------|--------|---------|--------------------------------------------------|
| `scenario`   | string | `LOW`   | Scenario name as defined in `scenerys.yaml`      |
| `iterations` | int    | `1500`  | Maximum number of tree expansion iterations      |
| `step`       | int    | `50`    | Tree node step distance                          |

**Examples:**

```bash
# Run with default parameters (LOW scenario, 1500 iterations, step 50)
python3 run_simulation.py

# Run the LOW scenario with 5000 iterations and step distance of 25
python3 run_simulation.py LOW 5000 25

# Run the MEDIUM scenario with 10000 iterations and step distance of 10
python3 run_simulation.py MEDIUM 10000 10

# Run the HIGH scenario with 25000 iterations and step distance of 50
python3 run_simulation.py HIGH 25000 50
```

---

### `data_acquisition.py` — Batch Data Collection

Runs multiple simulations across different parameter combinations and saves
results to CSV files under `simulation_data/`. Configure the test parameters
directly inside the script:

```python
rep       = 30              # number of repetitions per configuration
it        = [5000, 10000]   # list of iteration counts to test
dists     = [10, 25, 50]    # list of step distances to test
scenarios = ["LOW",         # list of scenarios to test
             "MEDIUM",
             "HIGH"]
```

Then run:

```bash
python3 data_acquisition.py
```

Results are saved under:
```
simulation_data/
└── {iterations}_{step}_{scenario}/
    ├── final_budget_reward/
    │   └── results_rrt.csv      # budget and reward statistics per repetition
    ├── all_goals/
    │   └── all_goals_{rep}.csv  # all goals in the scenario
    ├── visited_goals/
    │   └── visited_goals_{rep}.csv  # goals collected in the solution path
    ├── tree_nodes/
    │   └── tree_nodes_{rep}.csv # all nodes from the merged outward trees
    ├── full_path/
    │   └── full_path_{rep}.csv  # complete solution path (start → goals → start)
    ├── path_print/
    │   └── path_{rep}.pdf       # visualization of the final solution path
    └── full_rrt/
        └── full_rrt_{rep}.pdf   # visualization of the full explored tree
```

---

## Scenario Configuration

Scenarios are defined in `scenerys.yaml`. Each scenario specifies a list of
goals (x, y, reward) and obstacle polygons (list of vertices):

```yaml
LOW:
  goals:
    - [96,  727, 126]   # x, y, reward
    - [117, 266, 300]
    # ...
  obstacles:
    - [[140, 50], [140, 95], [300, 95], [300, 50]]  # polygon vertices
    # ...

MEDIUM:
  goals:
    # ...
  obstacles:
    # ...
```

To add a new scenario, simply add a new top-level key to `scenerys.yaml`
following the same structure. The new scenario will be immediately available
as an argument to `run_simulation.py`.

---

## Output Format

### `results_rrt.csv`

| Column           | Description                              |
|------------------|------------------------------------------|
| `Repeticao`      | Repetition index                         |
| `Budget_Usado`   | Total travel distance used               |
| `Reward_Coletado`| Total reward collected                   |
| `Budget_Usado%`  | Fraction of total budget used (0 to 1)   |
| `Reward_Coletado%`| Fraction of available reward collected  |

### `full_path_{rep}.csv`

| Column            | Description                             |
|-------------------|-----------------------------------------|
| `Id`              | Node ID (`START` for start/end nodes)   |
| `X`               | X position                              |
| `Y`               | Y position                              |
| `Reward_Coletado` | Accumulated reward at this node         |
| `Budget_Usado`    | Accumulated travel cost at this node    |
