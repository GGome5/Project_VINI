import pandas as pd
import os

# 1. Defina a sua lista de iterações aqui
lista_iteracoes = [1500, 5000, 10000, 15000, 25000, 50000]
passos = [10, 25, 50, 100]
mapas = ["LOW", "MEDIUM", "HIGH"]

# 2. Defina o diretório base (ajuste se necessário)
this_dir = os.path.dirname(os.path.abspath(__file__))
base_dir = os.path.join(this_dir, 'simulation_data')

best_run = [0,0,0,0,0,0]

for mapa in mapas:
    best_run = [0,0,0,0,0,0]
    for passo in passos:
        # print("\\midrule\n")
        # print(f"\\multirow{{5}}{{*}}{{\\rotatebox{{90}}{{{mapa} - {passo}}}}}")
        for iteracoes in lista_iteracoes:
            # 3. Monta o caminho dinâmico
            # Exemplo: data_analises/100_10_LOW/final_budget_reward/results_rrt.csv
            file_path = os.path.join(base_dir, f"{iteracoes}_{passo}_{mapa}", "final_budget_reward", "results_rrt.csv")
            
            # print(f"--- Analisando: {iteracoes} iterações ---")
            
            try:
                df = pd.read_csv(file_path)
                
                mean_budg_perc = df['Budget_Usado%'].mean()
                std_budg_perc  = df['Budget_Usado%'].std()
                mean_rewa_perc = df['Reward_Coletado%'].mean()
                std_rewa_perc  = df['Reward_Coletado%'].std()

                df_sorted = df.sort_values(by=['Reward_Coletado%', 'Budget_Usado%'], ascending=[False, True])
                best = df_sorted.iloc[0]

                best_rewa_perc = best['Reward_Coletado%']
                best_budg_perc = best['Budget_Usado%']
                best_rewa_id   = best['Repeticao']

                if (best_rewa_perc > best_run[5]) or (best_rewa_perc == best_run[5] and best_budg_perc < best_run[4]):
                    best_run = [iteracoes, passo, mapa, best_rewa_id, best_budg_perc, best_rewa_perc]
                
                # print(f"Media do budget       : {mean_budg_perc:.4f}%")
                # print(f"Desvio padrao budget  : {std_budg_perc:.4f}%")
                # print(f"Media da recompensa   : {mean_rewa_perc:.4f}%")
                # print(f"Desvio padrao reward  : {std_rewa_perc:.4f}%")
                
                # Formato para copiar direto para a sua tabela
                # print(f"& {iteracoes} &{mean_budg_perc:.3f} & {std_budg_perc:.3f} & {mean_rewa_perc:.3f} & {std_rewa_perc:.3f} & \\\\ \n")
                # print(f"[{mapa} | Passo: {passo:3d} | Iter: {iteracoes:5d}] -> Rep: {best_rewa_id} | Recompensa: {best_rewa_perc:.3f} | Budget: {best_budg_perc:.3f}")
                
            except FileNotFoundError:
                print(f"⚠️ Arquivo não encontrado: {file_path}\n")
            except KeyError as e:
                print(f"⚠️ Coluna não encontrada no arquivo {file_path}: {e}\n")

    for info in best_run:
        print(info, end=" ")
    print()