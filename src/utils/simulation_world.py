import numpy as np
import copy
from src.utils.agents import Lider 

class SimulationWorld:
    """Contenedor para la simulación. Gestion1 agentes, paredes y salidas."""
    def __init__(self, width, height, title="Simulación"):
        self.width = width
        self.height = height
        self.title = title
        self.agents = []
        self.walls = []
        self.exits = []
        self.exits_goals = [] 
        self.current_time = 0.0
        
        self.agents_active = 0      
        self.total_agents = 0       
        self.agents_evacuated = 0   
        
        self.time_to_50_percent = -1.0
        self.time_to_75_percent = -1.0
        self.time_to_90_percent = -1.0

    def add_wall_segment(self, p1, p2):
        self.walls.append((np.array(p1), np.array(p2)))

    def add_exit(self, p1, p2, goal_point):
        self.exits.append((np.array(p1), np.array(p2)))
        self.exits_goals.append(np.array(goal_point))

    def add_agent(self, agent):
        self.agents.append(agent)
        self.agents_active += 1
        self.total_agents += 1 

    def update(self, dt):
        """Actualiza el estado de todos los agentes en el mundo."""
        self.current_time += dt
        
        agents_to_update = copy.copy(self.agents)

        for agent in agents_to_update:
            if agent.estado != "EVACUADO":
                agent.update(dt, self.agents, self)
        
        for agent in agents_to_update:
            if agent.estado != "EVACUADO":
                if agent.is_evacuated(self.exits, self.current_time, self):
                    self.agents_active -= 1
                    self.agents_evacuated += 1
                    
                    if isinstance(agent, Lider) and hasattr(agent, 'estado_lider') and agent.estado_lider == "LIDERANDO" and agent.target_dependent:
                        agent.target_dependent.lider = None
                    
                    # print(f"Agente {agent.id} evacuado en {self.current_time:.2f}s")
                    
                    if self.total_agents > 0:
                        percent_evacuated = self.agents_evacuated / self.total_agents
                        
                        if self.time_to_50_percent < 0 and percent_evacuated >= 0.50:
                            self.time_to_50_percent = self.current_time
                        if self.time_to_75_percent < 0 and percent_evacuated >= 0.75:
                            self.time_to_75_percent = self.current_time
                        if self.time_to_90_percent < 0 and percent_evacuated >= 0.90:
                            self.time_to_90_percent = self.current_time
                    
        return self.agents_active > 0

    def get_evacuation_stats(self):
        times = [a.evacuation_time for a in self.agents if a.evacuation_time > 0]
        return times
