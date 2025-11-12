import numpy as np
from .constants import *

class Agent:
    """Clase base para todos los agentes. Implementa la física SFM."""
    def __init__(self, id, pos, radius=AGENT_RADIUS):
        self.id = id
        self.pos = np.array(pos, dtype=float)
        self.vel = np.array([0.0, 0.0], dtype=float)
        self.acc = np.array([0.0, 0.0], dtype=float)
        self.radius = radius
        self.mass = 60.0
        self.goal = None
        self.estado = "ACTIVO"
        self.color = "#000000"
        self.evacuation_time = -1.0 
        self.desired_speed = DESIRED_SPEED
        
        # --- ATRIBUTOS PARA EVASIÓN ---
        self.evade_timer = 0.0          # Cuánto tiempo (segundos) falta para terminar la evasión
        self.current_temp_goal = None   # El waypoint temporal para evadir
        
        self.stuck_check_timer = 0.0    # Temporizador para no chequear en cada frame
        self.last_dist_to_goal = np.inf # La distancia al objetivo en el último check
    
        
    def calculate_desired_force(self):
        """Calcula la fuerza que empuja al agente hacia su objetivo."""
        if self.goal is None:
            return np.array([0.0, 0.0])

        direction_to_goal = self.goal - self.pos
        norm_direction = np.linalg.norm(direction_to_goal)

        # Ya llegó a la meta
        if norm_direction < self.radius:
            return np.array([0.0, 0.0])

        e_i = direction_to_goal / norm_direction
        desired_velocity = e_i * self.desired_speed
        force = (desired_velocity - self.vel) * self.mass / RELAX_TIME
        return force
    
    def calculate_interaction_force(self, other_agent):
        """
        Calcula la fuerza de interacción TOTAL (social + física) con otro agente.
        """
        distance_vec = self.pos - other_agent.pos
        distance = np.linalg.norm(distance_vec)
        total_radius = self.radius + other_agent.radius
        
        # distance_vec normalizado
        e_ij = distance_vec / distance if distance > 0 else np.random.rand(2) - 0.5
        
        # --- 1. Fuerza de Repulsión Social (Psicológica) ---
        f_social_mag = REPULSION_STRENGTH * np.exp((total_radius - distance) / REPULSION_RANGE)
        f_social = f_social_mag * e_ij

        # --- 2. Fuerza de Contacto Físico (Empujón) ---
        overlap = total_radius - distance
        
        if overlap > 0:
            # Fuerza de "resorte" (body force) - Ley de Hooke
            f_body_mag = K_BODY * overlap
            
            # Fuerza de "Amortiguación" (Damping) - Absorbe el rebote
            # Proyecta la diferencia de velocidad sobre el vector *normal* (e_ij)
            vel_diff = other_agent.vel - self.vel
            f_damping_mag = C_DAMPING * overlap * np.dot(vel_diff, e_ij)
            f_damping = f_damping_mag * e_ij
            
            # Fuerza de "fricción" (sliding friction) - Amortigua el deslizamiento lateral
            t_ij = np.array([-e_ij[1], e_ij[0]]) # Vector tangencial
            f_friction_mag = K_FRICTION * overlap * np.dot(vel_diff, t_ij)
            f_friction = f_friction_mag * t_ij
            
            # La fuerza física total es la suma de las tres
            f_physical = (f_body_mag * e_ij) + f_damping + f_friction
        else:
            f_physical = np.array([0.0, 0.0])

        # La fuerza neta es la suma de la social y la física
        return f_social + f_physical
    
    
    # --- VERSIÓN FÍSICA DE REPELISIÓN DE PAREDES ---
    def calculate_wall_repulsion(self, walls):
        """Calcula la fuerza de repulsión FÍSICA y SOCIAL de TODAS las paredes."""
        total_force = np.array([0.0, 0.0])
        for wall in walls:
            p1 = wall[0]
            p2 = wall[1]
            
            # Encontrar el punto más cercano en el segmento de la pared
            line_vec = p2 - p1
            line_len_sq = np.dot(line_vec, line_vec)
            
            if line_len_sq == 0:
                closest_point = p1
            else:
                agent_vec = self.pos - p1
                t = np.dot(agent_vec, line_vec) / line_len_sq
                t_clamped = np.clip(t, 0, 1) 
                closest_point = p1 + t_clamped * line_vec
            
            dist_vec = self.pos - closest_point
            distance = np.linalg.norm(dist_vec)
            
            # (Esta es la normal, apuntando de la pared al agente)
            n_iw = dist_vec / distance if distance > 0 else np.random.rand(2) - 0.5
            
            # --- 1. Fuerza Social (Psicológica) de la Pared ---
            f_social_mag = WALL_REPULSION_STRENGTH * np.exp((self.radius - distance) / WALL_REPULSION_RANGE)
            f_social = f_social_mag * n_iw
            
            # --- 2. Fuerza Física (Colisión) con la Pared ---
            overlap = self.radius - distance
            if overlap > 0:
                # Fuerza de "resorte"
                f_body_mag = K_BODY * overlap
                
                # Fuerza de "fricción" (agente deslizándose por la pared)
                t_iw = np.array([-n_iw[1], n_iw[0]]) # Tangente de la pared
                f_friction_mag = K_FRICTION * overlap * np.dot(self.vel, t_iw)
                f_friction = -f_friction_mag * t_iw # Se opone al movimiento
                
                f_physical = (f_body_mag * n_iw) + f_friction
            else:
                f_physical = np.array([0.0, 0.0])

            total_force += (f_social + f_physical)
                
        return total_force

    def update_state_machine(self, all_agents, world):
        """Método 'virtual' para la lógica de IA. Se sobreescribe en las subclases."""
        pass

    def update(self, dt, all_agents, world):
        """Actualiza la física del agente."""
        if self.estado == "EVACUADO":
            self.vel = np.array([0.0, 0.0])
            return

        self.update_state_machine(all_agents, world)

        F_desired = self.calculate_desired_force()
        
        F_agents = np.array([0.0, 0.0])
        for other in all_agents:
            if self.id != other.id and other.estado != "EVACUADO":
                F_agents += self.calculate_interaction_force(other)
                
        F_walls = self.calculate_wall_repulsion(world.walls)

        F_net = F_desired + F_agents + F_walls
        
        self.acc = F_net / self.mass
        self.vel += self.acc * dt
        
        # Limitar velocidad
        speed = np.linalg.norm(self.vel)
        max_speed = self.desired_speed * 1.5
        if speed > max_speed:
             self.vel = (self.vel / speed) * max_speed
             
        self.pos += self.vel * dt

    def is_evacuated(self, exits, current_time, world):
        """Comprueba si el agente ha cruzado una salida."""
        if self.estado == "EVACUADO":
            return True
        
        for ex in exits:
            p1 = ex[0]
            p2 = ex[1]
            # Salida vertical (borde derecho)
            if p1[0] == p2[0] and p1[0] == world.width: 
                if self.pos[0] > world.width:
                    if self.pos[1] > min(p1[1], p2[1]) and self.pos[1] < max(p1[1], p2[1]):
                        self.evacuate(current_time)
                        return True
            
        return False
        
    def evacuate(self, current_time):
        self.estado = "EVACUADO"
        self.vel = np.array([0.0, 0.0])
        self.color = COLOR_MAP["EVACUADO"]
        self.evacuation_time = current_time

class Independiente(Agent):
    """Agente que prefiere su puerta de entrada, pero considera otras."""
    def __init__(self, id, pos, entry_goal):
        super().__init__(id, pos)
        self.preferred_goal = entry_goal
        self.goal = entry_goal
        self.color = COLOR_MAP["INDEPENDIENTE"]
        self.last_goal_check = 0.0

    def update_state_machine(self, all_agents, world):
            dt = TIME_STEP 
            
            # --- 1. LÓGICA DE EVASIÓN ---
            if self.evade_timer > 0:
                self.evade_timer -= dt
                self.goal = self.current_temp_goal
                if self.evade_timer <= 0:
                    self.current_temp_goal = None
                    self.last_dist_to_goal = np.inf 
                    self.stuck_check_timer = 0.0
                return 

            # --- 2. DETECTOR DE ATASCO ---
            self.stuck_check_timer += dt
            
            if self.goal is not None and self.stuck_check_timer > 0.5:
                current_dist_to_goal = np.linalg.norm(self.pos - self.goal)
                progress = self.last_dist_to_goal - current_dist_to_goal
                
                if progress < 0.1:
                    vec_to_goal = self.goal - self.pos
                    norm_vec_to_goal = vec_to_goal / (np.linalg.norm(vec_to_goal) + 1e-5)
                    vec_perp = np.array([-norm_vec_to_goal[1], norm_vec_to_goal[0]])
                    direction = np.random.choice([-1, 1]) 
                    self.current_temp_goal = self.pos + (vec_perp * direction * 3.0) + (norm_vec_to_goal * 1.0)
                    self.goal = self.current_temp_goal
                    self.evade_timer = 2.0 
                
                self.last_dist_to_goal = current_dist_to_goal
                self.stuck_check_timer = 0.0
                
                if self.evade_timer > 0:
                    return
                    
            # --- 3. LÓGICA DE ESTADO NORMAL ---
            
            # 3.1. Determinar el objetivo final (Salida)
            # (Esta lógica es la de antes, para elegir entre múltiples salidas)
            final_goal = self.preferred_goal
            if len(world.exits_goals) > 1 and (world.current_time - self.last_goal_check > 1.0):
                self.last_goal_check = world.current_time
                costs = []
                for goal in world.exits_goals:
                    dist = np.linalg.norm(self.pos - goal)
                    if not np.array_equal(goal, self.preferred_goal):
                        dist *= 1.5 # Penalización
                    costs.append(dist)
                best_goal_idx = np.argmin(costs)
                final_goal = world.exits_goals[best_goal_idx]

            # 3.2. Lógica de Waypoint
            self.goal = final_goal # Asumir el objetivo final por defecto
            
            if world.waypoints:
                for wp in world.waypoints:
                    # Si estamos a la izquierda de la pared interna (zona de waypoint)
                    if self.pos[0] < wp['zone_x_limit']:
                        self.goal = wp['goal'] # Anular el objetivo
                        break # Encontramos nuestro waypoint, no necesitamos buscar más
                              
class Dependiente(Agent):
    """Agente que no se mueve hasta que un Lider lo guía."""
    def __init__(self, id, pos):
        super().__init__(id, pos)
        self.estado = "DEPENDIENTE_ESPERA"
        self.color = COLOR_MAP["DEPENDIENTE_ESPERA"]
        self.lider = None
        self.desired_speed = 0.0 

    def update_state_machine(self, all_agents, world):
        """Lógica de estado corregida para el Dependiente."""
        if self.estado == "DEPENDIENTE_LIDERADO":
            if self.lider and self.lider.estado != "EVACUADO":
                self.goal = self.lider.goal
                self.desired_speed = self.lider.desired_speed * 0.8
                self.color = COLOR_MAP["DEPENDIENTE_LIDERADO"]
            else:
                self.lider = None 
                self.estado = "DEPENDIENTE_SOLO" 
                self.color = COLOR_MAP["DEPENDIENTE_SOLO"]
                self.desired_speed = DESIRED_SPEED * 0.5
                
                dists = [np.linalg.norm(self.pos - goal) for goal in world.exits_goals]
                if dists:
                    self.goal = world.exits_goals[np.argmin(dists)]
        
        elif self.estado == "DEPENDIENTE_SOLO":
            if world.current_time - getattr(self, 'last_goal_check', 0) > 2.0:
                self.last_goal_check = world.current_time
                dists = [np.linalg.norm(self.pos - goal) for goal in world.exits_goals]
                if dists:
                    self.goal = world.exits_goals[np.argmin(dists)]


class Lider(Agent):
    """Agente que busca un Dependiente y lo guía a la salida."""
    def __init__(self, id, pos):
        super().__init__(id, pos)
        self.estado_lider = "BUSCANDO" 
        self.target_dependent = None
        self.color = COLOR_MAP["LIDER"]

    def find_nearest_exit_goal(self, world):
        if not world.exits_goals:
            return np.array([world.width / 2, world.height / 2]) 
        
        dists = [np.linalg.norm(self.pos - goal) for goal in world.exits_goals]
        return world.exits_goals[np.argmin(dists)]

    def update_state_machine(self, all_agents, world):
        dt = TIME_STEP 
        HANDOFF_DISTANCE = 5.0 

        # --- 1. LÓGICA DE EVASIÓN ---
        if self.evade_timer > 0:
            self.evade_timer -= dt
            self.goal = self.current_temp_goal
            if self.evade_timer <= 0:
                self.current_temp_goal = None
                self.last_dist_to_goal = np.inf 
                self.stuck_check_timer = 0.0
            return 

        # --- 2. DETECTOR DE ATASCO ---
        self.stuck_check_timer += dt
        
        is_near_target = False
        if hasattr(self, 'estado_lider') and self.estado_lider == "LIDERANDO_HACIA_OBJETIVO" and self.target_dependent:
             if np.linalg.norm(self.pos - self.target_dependent.pos) < 1.0:
                 is_near_target = True

        if self.goal is not None and self.stuck_check_timer > 0.5 and not is_near_target:
            current_dist_to_goal = np.linalg.norm(self.pos - self.goal)
            progress = self.last_dist_to_goal - current_dist_to_goal
            
            if progress < 0.1: 
                vec_to_goal = self.goal - self.pos
                norm_vec_to_goal = vec_to_goal / (np.linalg.norm(vec_to_goal) + 1e-5)
                vec_perp = np.array([-norm_vec_to_goal[1], norm_vec_to_goal[0]])
                direction = np.random.choice([-1, 1])
                self.current_temp_goal = self.pos + (vec_perp * direction * 3.0) + (norm_vec_to_goal * 1.0)
                self.goal = self.current_temp_goal
                self.evade_timer = 2.0 
            
            self.last_dist_to_goal = current_dist_to_goal
            self.stuck_check_timer = 0.0
            
            if self.evade_timer > 0:
                 return
                 
        # --- 3. LÓGICA DE ESTADO PRINCIPAL (MODIFICADA CON WAYPOINTS) ---
        
        mission_incomplete = False
        for agent in all_agents:
            if isinstance(agent, Dependiente) and \
               (agent.estado == "DEPENDIENTE_ESPERA" or agent.estado == "DEPENDIENTE_LIDERADO"):
                mission_incomplete = True
                break

        # Función helper para encontrar el objetivo (waypoint o salida)
        def get_nav_goal():
            final_goal = world.exits_goals[np.argmin([np.linalg.norm(self.pos - g) for g in world.exits_goals])]
            nav_goal = final_goal
            if world.waypoints:
                for wp in world.waypoints:
                    if self.pos[0] < wp['zone_x_limit']:
                        nav_goal = wp['goal'] # Anular
                        break
            return nav_goal

        # --- A. Si la misión está COMPLETA ---
        if not mission_incomplete:
            if self.estado_lider != "EVACUANDO_FINAL":
                print(f"Lider {self.id}: ¡Misión cumplida! Evacuando...")
                self.estado_lider = "EVACUANDO_FINAL"
                self.last_dist_to_goal = np.inf
            
            self.goal = get_nav_goal() # Usar el waypoint si es necesario para salir
            return 

        # --- B. Si la misión AÚN NO TERMINA ---
        
        if self.estado_lider == "LIDERANDO_HACIA_OBJETIVO":
            if not self.target_dependent or self.target_dependent.estado != "DEPENDIENTE_ESPERA":
                self.estado_lider = "BUSCANDO"
                self.target_dependent = None
                self.last_dist_to_goal = np.inf
            else:
                self.goal = self.target_dependent.pos # Objetivo: el dependiente
                dist_to_target = np.linalg.norm(self.pos - self.target_dependent.pos)
                if dist_to_target < (self.radius + self.target_dependent.radius + 0.5):
                    self.estado_lider = "LIDERANDO"
                    self.target_dependent.lider = self
                    self.target_dependent.estado = "DEPENDIENTE_LIDERADO"
                    self.last_dist_to_goal = np.inf

        elif self.estado_lider == "LIDERANDO":
            if not self.target_dependent or self.target_dependent.estado == "EVACUADO":
                self.estado_lider = "BUSCANDO"
                self.target_dependent = None
                self.last_dist_to_goal = np.inf
            else:
                nav_goal = get_nav_goal() # Objetivo de navegación (waypoint o salida)
                self.goal = nav_goal 
                
                # Lógica de "entrega" (basada en la salida real, no el waypoint)
                final_exit_goal = world.exits_goals[np.argmin([np.linalg.norm(self.pos - g) for g in world.exits_goals])]
                dist_to_exit = np.linalg.norm(self.target_dependent.pos - final_exit_goal)
                
                if dist_to_exit < HANDOFF_DISTANCE:
                    self.target_dependent.estado = "DEPENDIENTE_SOLO" 
                    self.target_dependent.lider = None 
                    self.target_dependent = None
                    self.estado_lider = "BUSCANDO" 
                    self.last_dist_to_goal = np.inf

        if self.estado_lider == "BUSCANDO":
            best_target = None
            min_dist = np.inf
            for agent in all_agents:
                if isinstance(agent, Dependiente) and agent.estado == "DEPENDIENTE_ESPERA":
                    dist = np.linalg.norm(self.pos - agent.pos)
                    if dist < min_dist:
                        min_dist = dist
                        best_target = agent
            
            if best_target:
                self.target_dependent = best_target
                self.estado_lider = "LIDERANDO_HACIA_OBJETIVO"
                self.goal = best_target.pos
                self.last_dist_to_goal = np.inf
            else:
                # Patrullar (ir al centro, usando el waypoint si aplica)
                self.goal = get_nav_goal()
                self.last_dist_to_goal = np.inf
    
    