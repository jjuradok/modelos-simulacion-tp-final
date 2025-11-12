# --- MODO DE SIMULACIÓN ---
# Cambia esta constante para alternar entre los perfiles de comportamiento
# Opciones: "RACIONAL" o "EMERGENCIA"
SIMULATION_MODE = "EMERGENCIA" 


# --- Parámetros de Simulación y Agente ---
TIME_STEP = 0.02       
AGENT_RADIUS = 0.3

# --- Constantes de Colisión Física (Independientes del modo) ---
K_BODY = 1.2e5      # Dureza del cuerpo
K_FRICTION = 2.4e5  # Fricción
C_DAMPING = 1.0e5   # Amortiguación 

# --- PARÁMETROS DEPENDIENTES DEL MODO ---

if SIMULATION_MODE == "RACIONAL":
    # --- MODO RACIONAL / CALMADO ---
    # Los agentes se mueven a velocidad de caminata y respetan el espacio personal.
    
    DESIRED_SPEED = 1.3       # Velocidad de caminata normal (m/s)
    RELAX_TIME = 0.6          # Tiempo de reacción más relajado
    REPULSION_STRENGTH = 2.0  # (ALTO) Mucha fuerza para mantener distancia
    REPULSION_RANGE = 1.2     # (ALTO) Espacio personal amplio
    
    # Fuerzas de pared para el modo racional
    WALL_REPULSION_STRENGTH = 4.0 # (ALTO) Evitan activamente las paredes
    WALL_REPULSION_RANGE = 0.8    # (ALTO) Detectan paredes desde más lejos

elif SIMULATION_MODE == "EMERGENCIA":
    # --- MODO EMERGENCIA / PÁNICO (Fenómeno FIS) ---
    # Los agentes intentan ir "más rápido" y se "empujan", 
    # reduciendo su espacio personal.
    
    DESIRED_SPEED = 2.5       # (ALTO) Mayor "fuerza de deseo"
    RELAX_TIME = 0.3          # (BAJO) Intentan acelerar muy rápido
    REPULSION_STRENGTH = 0.5  # (BAJO) Menos respeto por el espacio, más "empujones"
    REPULSION_RANGE = 0.7     # (BAJO) Toleran estar mucho más cerca de otros
    
    # Fuerzas de pared para el modo emergencia
    WALL_REPULSION_STRENGTH = 3.0 # (Original) Fuerza de pared estándar
    WALL_REPULSION_RANGE = 0.5    # (Original) Detectan paredes a la distancia normal

else:
    raise ValueError(f"Modo de simulación '{SIMULATION_MODE}' no reconocido.")

# --- Fuerzas de evasión (obstáculos) ---
EVADE_X = 1
EVADE_Y = 1/3
EVADE_TIME = 2.0

# Colores de estado
COLOR_MAP = {
    "INDEPENDIENTE": "#0077BE", 
    "LIDER": "#009E73",         
    "DEPENDIENTE_ESPERA": "#D55E00", 
    "DEPENDIENTE_LIDERADO": "#F0E442", 
    "DEPENDIENTE_SOLO": "#882255", 
    "EVACUADO": "#CCCCCC"       
}

# --- Configuración de la Población ---
N_INDEPENDIENTES = 80 # Número de agentes independientes por defecto
N_LIDERES = 10       # Número de líderes por defecto
N_DEPENDIENTES = 10    # Número de dependientes por defecto

# Configuración de población para escenarios especiales
N_INDEPENDIENTES_ESC3 = 100
N_LIDERES_ESC3 = 0
N_DEPENDIENTES_ESC3 = 0 # <-- (CORREGIDO)

EXIT_WIDTH = 2.5  # Ancho estándar de las salidas (metros)