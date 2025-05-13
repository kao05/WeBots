# my_controller.py

# Importamos las clases necesarias del módulo controller de Webots
from controller import Robot, Motor, DistanceSensor
import random

# Creamos la instancia del robot
robot = Robot()

# Obtenemos el tamaño del paso de simulación (timestep)
timestep = int(robot.getBasicTimeStep())

# Definimos la velocidad máxima de las ruedas (en rad/s)
MAX_SPEED = 3.28

# --------------------------------------------------------------------
# Inicialización de motores
# --------------------------------------------------------------------
left_wheel  = robot.getDevice('left wheel motor')
right_wheel = robot.getDevice('right wheel motor')
left_wheel.setPosition(float('inf'))
right_wheel.setPosition(float('inf'))
left_wheel.setVelocity(0.0)
right_wheel.setVelocity(0.0)

# --------------------------------------------------------------------
# Inicialización de sensores de proximidad
# --------------------------------------------------------------------
sensor_right = robot.getDevice('ps0')
sensor_left  = robot.getDevice('ps7')
sensor_right.enable(timestep)
sensor_left.enable(timestep)

# --------------------------------------------------------------------
# Parámetros para el comportamiento de evitación
# --------------------------------------------------------------------
GAIN = 0.0005                   # Ganancia Braitenberg
CLEAR_THRESHOLD = 50.0       # Umbral para espacio libre

# --------------------------------------------------------------------
# Variables para exploración y detección de bucles
# --------------------------------------------------------------------
explore_steps = 0            # Pasos restantes en la dirección actual
explore_left_speed = 0.0     # Velocidad izquierda exploración
explore_right_speed = 0.0    # Velocidad derecha exploración
loop_count = 0               # Contador de exploraciones consecutivas
LOOP_LIMIT = 5               # Veces que agotar exploración sin choque antes de giro forzado
pivot_steps = 0              # Pasos restantes para el giro forzado
PIVOT_DURATION = 30          # Duración (pasos) del giro forzado

# --------------------------------------------------------------------
# Bucle principal de control
# --------------------------------------------------------------------
while robot.step(timestep) != -1:
    # Leer sensores
    val_r = sensor_right.getValue()
    val_l = sensor_left.getValue()

    # Si estamos en giro forzado
    if pivot_steps > 0:
        # Giro 180° rápido para romper bucle
        left_wheel.setVelocity(MAX_SPEED)
        right_wheel.setVelocity(-MAX_SPEED)
        pivot_steps -= 1
        continue

    # Evitación de obstáculos detectados
    if val_l > CLEAR_THRESHOLD or val_r > CLEAR_THRESHOLD:
        # Reseteamos exploración y contador de bucles
        explore_steps = 0
        loop_count = 0
        # Braitenberg básico
        sp_l = MAX_SPEED - GAIN * val_l
        sp_r = MAX_SPEED - GAIN * val_r
    else:
        # Zona libre: exploración aleatoria
        if explore_steps <= 0:
            # Nueva directiva exploratoria
            explore_left_speed  = random.uniform(0.5 * MAX_SPEED, MAX_SPEED)
            explore_right_speed = random.uniform(0.5 * MAX_SPEED, MAX_SPEED)
            explore_steps = random.randint(20, 100)
            loop_count += 1
            # Si hemos agotado exploraciones sin choque varias veces seguidas
            if loop_count >= LOOP_LIMIT:
                pivot_steps = PIVOT_DURATION
                loop_count = 0
                continue  # saltamos a la rutina de giro en siguiente iteración
        sp_l = explore_left_speed
        sp_r = explore_right_speed
        explore_steps -= 1

    # Saturar velocidades en [0, MAX_SPEED]
    sp_l = max(0.0, min(MAX_SPEED, sp_l))
    sp_r = max(0.0, min(MAX_SPEED, sp_r))

    # Enviar velocidades
    left_wheel.setVelocity(sp_l)
    right_wheel.setVelocity(sp_r)