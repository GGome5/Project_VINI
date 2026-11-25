import time
import pygame
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

VEL_MAX = 20.0      # Velocidade máxima (rad/s)
ANGULO_MAX = 0.78   # Limite da direção (~45 graus)
DEADZONE = 0.15     # Zona morta do analógico


def aplicar_deadzone(valor, limite=DEADZONE):
    return 0.0 if abs(valor) < limite else valor


def main():
    # Inicializa a manete
    pygame.init()
    pygame.joystick.init()

    if pygame.joystick.get_count() == 0:
        raise RuntimeError("Nenhuma manete encontrada! Plugue o controle e tente novamente.")

    joy = pygame.joystick.Joystick(0)
    joy.init()
    print(f"Controle conectado: {joy.get_name()}")

    # Conecta ao CoppeliaSim
    client = RemoteAPIClient()
    sim = client.require('sim')

    # Localiza as juntas na cena atual (/base_link_respondable ou /VINI)
    try:
        motor = sim.getObject('/base_link_respondable/rear_wheel_joint')
        steer = sim.getObject('/base_link_respondable/steering_joint')
    except Exception:
        motor = sim.getObject('/VINI/rear_wheel_joint')
        steer = sim.getObject('/VINI/steering_joint')

    if sim.getSimulationState() == sim.simulation_stopped:
        sim.startSimulation()
        print("Simulação iniciada automaticamente no CoppeliaSim!")

    print(f"Conectado ao CoppeliaSim com sucesso! (motor={motor}, steer={steer})")
    print("Dirigindo! Pressione Ctrl+C no terminal para encerrar.")

    try:
        while True:
            pygame.event.pump()

            eixo_direcao = aplicar_deadzone(joy.get_axis(0))
            eixo_aceleracao = aplicar_deadzone(joy.get_axis(1))

            vel = -eixo_aceleracao * VEL_MAX
            angulo = -eixo_direcao * ANGULO_MAX

            sim.setJointTargetVelocity(motor, vel)
            sim.setJointTargetPosition(steer, angulo)

            time.sleep(0.02)  # 50 Hz

    except KeyboardInterrupt:
        sim.setJointTargetVelocity(motor, 0.0)
        sim.setJointTargetPosition(steer, 0.0)
        print("\nControle encerrado e robô parado.")


if __name__ == "__main__":
    main()
