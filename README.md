# Projeto VINI — Robô Móvel (Projeto Final)

Repositório do projeto final da disciplina contendo a modelagem mecânica 3D (SolidWorks), as cenas de simulação (CoppeliaSim) e os códigos de teleoperação e controle via manete USB (Python / Jupyter Notebook) do robô **VINI**.

---

## Estrutura do Projeto

```text
Projeto_VINI/
├── cad_solidworks/                 # Peças (.SLDPRT) e montagem completa (.SLDASM) no SolidWorks
│   ├── Montagem_VINI.SLDASM        # Montagem principal do robô VINI
│   └── *.SLDPRT                    # Peças do chassi, direção, rodas, hubs e servo MG996R
├── simulacao_coppeliasim/          # Cenas de simulação do CoppeliaSim (.ttt)
│   ├── Cena_Cidade-VINI.ttt        # Cena urbana completa com o robô VINI
│   └── Novo_modelo_Cena.ttt        # Cena do modelo atualizado do VINI
├── controle/                       # Teleoperação via manete USB (Xbox / Gamepad) via ZMQ Remote API
│   ├── controle_manete_vini.ipynb  # Jupyter Notebook interativo (conexão e loop separados)
│   └── controle_manete_vini.py     # Script Python standalone para execução direta no terminal
├── requirements.txt                # Dependências Python para o controle do robô
└── .gitignore                      # Ignora arquivos temporários do SolidWorks (~$*), Jupyter e Python
```

> **Nota sobre o SolidWorks:** Todas as peças (`.SLDPRT`) foram mantidas na mesma pasta que a montagem (`Montagem_VINI.SLDASM`) dentro de `cad_solidworks/` para preservar automaticamente todas as referências de montagem ao abrir o arquivo.

---

## Como Executar o Controle via Manete no CoppeliaSim

### 1. Instalar as dependências
No terminal, instale as bibliotecas necessárias:

```bash
pip install -r requirements.txt
```

### 2. Abrir a cena no CoppeliaSim
1. Abra o **CoppeliaSim**.
2. Carregue uma das cenas em `simulacao_coppeliasim/` (`Novo_modelo_Cena.ttt` ou `Cena_Cidade-VINI.ttt`).
3. Conecte sua manete USB (ex.: controle de Xbox 360/One) no computador.

### 3. Executar o controle
Você pode dirigir o robô de duas formas:

- **Via Jupyter Notebook (recomendado para ajustar velocidades e parâmetros em tempo real):**
  Abra `controle/controle_manete_vini.ipynb` e execute as células de conexão e de loop de controle.
- **Via Terminal (execução direta):**
  ```bash
  python controle/controle_manete_vini.py
  ```
