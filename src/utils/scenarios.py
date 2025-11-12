# src/utils/scenarios.py

import numpy as np
from src.utils.simulation_world import SimulationWorld
from src.utils.agents import Independiente, Lider, Dependiente
# Importar las constantes de población y el radio del agente
from src.utils.constants import (
    AGENT_RADIUS,
    N_INDEPENDIENTES, N_LIDERES, N_DEPENDIENTES,
    N_INDEPENDIENTES_ESC3, N_LIDERES_ESC3, N_DEPENDIENTES_ESC3, EXIT_WIDTH
)

def create_scenario(scenario_id, populate=True): 
    """Fábrica de mundos. Crea el escenario basado en un ID."""
    
    W, H = 30.0, 20.0 
    
    waypoints = []
    spawn_check_zones = [] # Lista de zonas donde NO se puede spawnear

    
    if scenario_id == 1: 
        world = SimulationWorld(W, H, "Escenario 1: Una Salida Ancha (4m)")
        world.add_wall_segment((0, 0), (W, 0)) 
        world.add_wall_segment((0, 0), (0, H)) 
        world.add_wall_segment((0, H), (W, H)) 
        world.add_wall_segment((W, 0), (W, H/2 - (EXIT_WIDTH/2)))
        world.add_wall_segment((W, H/2 + (EXIT_WIDTH/2)), (W, H))
        world.add_exit((W, H/2 - (EXIT_WIDTH/2)), (W, H/2 + (EXIT_WIDTH/2)), (W, H/2)) 

    elif scenario_id == 12: 
        world = SimulationWorld(W, H, "Escenario 1: Dos Salidas Estrechas (2m c/u)")
        world.add_wall_segment((0, 0), (W, 0))
        world.add_wall_segment((0, 0), (0, H))
        world.add_wall_segment((0, H), (W, H))
        world.add_wall_segment((W, 0), (W, 5)) 
        world.add_wall_segment((W, 7), (W, H - 7)) 
        world.add_wall_segment((W, H - 5), (W, H))
        world.add_exit((W, 5), (W, 7), (W, 6)) 
        world.add_exit((W, H - 7), (W, H - 5), (W, H - 6)) 
        
    elif scenario_id == 2: 
        world = create_scenario(1, populate=False)
        world.title = "Escenario 2: Salida Central"
    
    elif scenario_id == 22: 
        world = SimulationWorld(W, H, "Escenario 2: Salida en Esquina (4m)")
        world.add_wall_segment((0, 0), (W, 0))
        world.add_wall_segment((0, 0), (0, H))
        world.add_wall_segment((0, H), (W, H))
        world.add_wall_segment((W, 0), (W, H - 4))
        world.add_exit((W, H - 4), (W, H), (W, H - 2)) 

    elif scenario_id == 3: 
        world = create_scenario(1, populate=False)
        world.title = "Escenario 3: Planta Abierta"

    elif scenario_id == 32: 
        world = create_scenario(1, populate=False)
        world.title = "Escenario 3: Compartimentada"
        wall_x = W/2
        gap_y_start = H/3
        gap_y_end = H*2/3
        world.add_wall_segment((wall_x, 0), (wall_x, gap_y_start))
        world.add_wall_segment((wall_x, gap_y_end), (wall_x, H))
        
        waypoint_pos = np.array([wall_x, (gap_y_start + gap_y_end) / 2])
        waypoints.append({'zone_x_limit': wall_x, 'goal': waypoint_pos})
        
        # Añadir la pared interna a las zonas de "no-spawneo"
        spawn_check_zones.append({'type': 'wall', 'x': wall_x})

        
    elif scenario_id == 4: 
        world = create_scenario(1, populate=False)
        world.title = "Escenario 4: Sin Obstáculo"
        
    elif scenario_id == 42: 
        world = create_scenario(1, populate=False)
        world.title = "Escenario 4: Con Obstáculo (FIS)"
        
        # --- (OBSTÁCULO CIRCULAR) ---
        col_x = W - 7 
        col_y = H / 2
        col_radius = 1.5  # Radio del pilar circular (1 metro)
        num_sides = 12    # Número de lados para aproximar el círculo (Dodecágono)
        
        # Generar los segmentos de pared para el pilar
        points = []
        for i in range(num_sides):
            angle = (i / num_sides) * 2 * np.pi
            x = col_x + col_radius * np.cos(angle)
            y = col_y + col_radius * np.sin(angle)
            points.append(np.array([x, y]))
        
        # Añadir los segmentos de pared al mundo
        for i in range(num_sides):
            p1 = points[i]
            p2 = points[(i + 1) % num_sides] # Conectar el último de vuelta al primero
            world.add_wall_segment(p1, p2)
        
        # Registrar la "zona prohibida" del pilar para el spawneo
        spawn_check_zones.append({
            'type': 'pillar', 
            'center': (col_x, col_y), 
            'radius': col_radius + AGENT_RADIUS # Radio del pilar + radio del agente
        })
       

    else:
        print(f"Error: Escenario {scenario_id} no reconocido.")
        return None # Corregido para devolver solo uno

    world.waypoints = waypoints

    # --- Bloque de población de agentes ---
    if populate:
        
        # 1. Usar constantes
        n_independientes = N_INDEPENDIENTES
        n_lideres = N_LIDERES
        n_dependientes = N_DEPENDIENTES
        
        # 2. Sobrescribir si es el escenario 3 o 32
        if (scenario_id == 3 or scenario_id == 32):
            n_independientes = N_INDEPENDIENTES_ESC3
            n_lideres = N_LIDERES_ESC3
            n_dependientes = N_DEPENDIENTES_ESC3
        
        # 3. Poblar
        agent_id = 0
        entry_doors = world.exits_goals if world.exits_goals else [np.array([W, H/2])]
        
        # (Lógica de Spawneo Corregida)
        
        for i in range(n_independientes):
            while True: # Buscar una posición válida
                pos = np.array([np.random.uniform(1, W-1), np.random.uniform(1, H-1)])
                
                is_valid = True
                for zone in spawn_check_zones:
                    if zone['type'] == 'wall' and (pos[0] > zone['x'] - AGENT_RADIUS and pos[0] < zone['x'] + AGENT_RADIUS):
                        is_valid = False; break
                    if zone['type'] == 'pillar':
                        dist_to_center = np.linalg.norm(pos - zone['center'])
                        if dist_to_center < zone['radius']:
                            is_valid = False; break
                
                if is_valid:
                    break 
            
            entry_goal = entry_doors[i % len(entry_doors)]
            agent = Independiente(agent_id, pos, entry_goal)
            world.add_agent(agent)
            agent_id += 1
            
        for i in range(n_lideres):
            while True: 
                pos = np.array([np.random.uniform(1, W-1), np.random.uniform(1, H-1)])
                
                is_valid = True
                for zone in spawn_check_zones:
                    if zone['type'] == 'wall' and (pos[0] > zone['x'] - AGENT_RADIUS and pos[0] < zone['x'] + AGENT_RADIUS):
                        is_valid = False; break
                    if zone['type'] == 'pillar':
                        dist_to_center = np.linalg.norm(pos - zone['center'])
                        if dist_to_center < zone['radius']:
                            is_valid = False; break
                
                if is_valid:
                    break 
            
            agent = Lider(agent_id, pos)
            world.add_agent(agent)
            agent_id += 1

        for i in range(n_dependientes):
            while True: 
                pos = np.array([np.random.uniform(1, W-1), np.random.uniform(1, H-1)])
                
                is_valid = True
                for zone in spawn_check_zones:
                    if zone['type'] == 'wall' and (pos[0] > zone['x'] - AGENT_RADIUS and pos[0] < zone['x'] + AGENT_RADIUS):
                        is_valid = False; break
                    if zone['type'] == 'pillar':
                        dist_to_center = np.linalg.norm(pos - zone['center'])
                        if dist_to_center < zone['radius']:
                            is_valid = False; break
                
                if is_valid:
                    break 
            
            agent = Dependiente(agent_id, pos)
            world.add_agent(agent)
            agent_id += 1
        
    return world # Devuelve un solo valor