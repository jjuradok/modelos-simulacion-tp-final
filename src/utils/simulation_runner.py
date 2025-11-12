# src/utils/simulation_runner.py
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
import matplotlib.animation as animation
import pandas as pd  # (NUEVO) Importar pandas

# (NUEVO) Intentar importar 'display' para notebooks, si no, usar 'print'
try:
    from IPython.display import display
except ImportError:
    display = print # display será un alias de print si IPython no está

from src.utils.constants import *
from src.utils.scenarios import create_scenario

def run_preview(world):
    """Modo 1: Ejecuta la simulación con visualización en tiempo real."""
    
    # --- (Sin cambios en esta función) ---
    
    fig, ax = plt.subplots(figsize=(world.width / 3, world.height / 3))
    ax.set_xlim(0, world.width + 3) 
    ax.set_ylim(0, world.height)
    ax.set_aspect('equal')
    ax.set_title(world.title)
    
    for wall in world.walls:
        ax.plot([wall[0][0], wall[1][0]], [wall[0][1], wall[1][1]], color='black', linewidth=2)
    for ex in world.exits:
        ax.plot([ex[0][0], ex[1][0]], [ex[0][1], ex[1][1]], color='green', linewidth=4, alpha=0.5)

    agent_circles = []
    for agent in world.agents:
        circle = Circle(agent.pos, agent.radius, color=agent.color, alpha=0.9)
        ax.add_patch(circle)
        agent_circles.append(circle)
        
    time_text = ax.text(0.02, 0.95, '', transform=ax.transAxes)
    agent_text = ax.text(0.02, 0.90, '', transform=ax.transAxes)
    
    def animate(frame):
        is_running = world.update(TIME_STEP)
        
        for i, agent in enumerate(world.agents):
            agent_circles[i].set_center(agent.pos)
            
            new_color = agent.color 
            if hasattr(agent, 'estado') and agent.estado == "DEPENDIENTE_SOLO":
                new_color = COLOR_MAP["DEPENDIENTE_SOLO"]
            elif hasattr(agent, 'estado') and agent.estado == "DEPENDIENTE_LIDERADO":
                 new_color = COLOR_MAP["DEPENDIENTE_LIDERADO"]
            elif agent.estado == "EVACUADO":
                new_color = COLOR_MAP["EVACUADO"]
                agent_circles[i].set_alpha(0.2)
            
            agent_circles[i].set_color(new_color)
        
        time_text.set_text(f'Tiempo: {world.current_time:.2f}s')
        agent_text.set_text(f'Agentes Activos: {world.agents_active}/{world.total_agents}')
        
        if not is_running:
            ani.event_source.stop()
            print("Simulación 'Preview' finalizada.")
            
        return agent_circles + [time_text, agent_text]

    ani = animation.FuncAnimation(fig, animate, frames=2000, 
                                  interval=TIME_STEP * 1000, blit=True)
    plt.show()


# --- (FUNCIÓN RUN_TEST COMPLETAMENTE REESCRITA) ---

def run_test(main_scenario_id, n_runs=10):
    """
    Modo 2: Ejecuta N corridas para AMBAS casuísticas del escenario principal (ej. 1 y 12)
    y muestra estadísticas comparativas.
    """
    
    print(f"--- Iniciando Modo Test Comparativo ---")
    print(f"Escenario Principal: {main_scenario_id}, Corridas por casuística: {n_runs}")
    
    # 1. Mapear el ID principal a sus dos sub-casos
    scenario_map = {
        1: [1, 12],
        2: [2, 22],
        3: [3, 32],
        4: [4, 42]
    }
    
    if main_scenario_id not in scenario_map:
        print(f"Error: ID de escenario principal no válido '{main_scenario_id}'. Debe ser 1, 2, 3, o 4.")
        return
        
    scenario_ids_to_run = scenario_map[main_scenario_id]
    
    # Diccionarios para guardar los datos de ambas corridas
    all_run_data = {}
    all_individual_times = {}
    
    max_time_limit = 120.0 
    
    # 2. Bucle principal: iterar sobre las dos casuísticas (ej. 1 y 12)
    for scenario_id in scenario_ids_to_run:
        
        # Obtener el título del escenario para usarlo como clave
        temp_world = create_scenario(scenario_id, populate=False) 
        if temp_world is None:
            print(f"Error al cargar escenario {scenario_id}")
            continue
        scenario_title = temp_world.title
        
        print(f"\n--- Corriendo Casuística: {scenario_title} (ID: {scenario_id}) ---")
        
        # Listas para guardar los datos de ESTA casuística
        t50_times, t75_times, t90_times, t100_times = [], [], [], []
        individual_times = []
        
        # Bucle de N corridas (como antes)
        for i in range(n_runs):
            world = create_scenario(scenario_id) 
            if world is None: continue 
            
            print(f"  Iniciando corrida {i+1}/{n_runs}...", end=" ", flush=True)
            
            is_running = True
            while is_running and world.current_time < max_time_limit:
                is_running = world.update(TIME_STEP)
            
            # Recolectar datos de la corrida
            run_times = world.get_evacuation_stats()
            individual_times.extend(run_times)
            
            if world.time_to_50_percent > 0: t50_times.append(world.time_to_50_percent)
            if world.time_to_75_percent > 0: t75_times.append(world.time_to_75_percent)
            if world.time_to_90_percent > 0: t90_times.append(world.time_to_90_percent)
            
            if not is_running: 
                t100_times.append(world.current_time)
                print(f"Finalizada en {world.current_time:.2f}s.")
            else:
                print(f"TIMEOUT. {world.agents_active} agentes no evacuaron.")
        
        # Guardar los datos agregados de esta casuística
        all_individual_times[scenario_title] = individual_times
        all_run_data[scenario_title] = {
            'T50': t50_times,
            'T75': t75_times,
            'T90': t90_times,
            'T100': t100_times,
        }
            
    if not all_run_data:
        print("No se registraron datos de simulación.")
        return

    # --- 3. Mostrar Tabla Comparativa (DataFrame) ---
    
    print("\n" + "="*50)
    print("  Resultados: Medias de Tiempos de Evacuación (segundos)")
    print("="*50)
    
    summary_data = {}
    for title, data in all_run_data.items():
        summary_data[title] = {
            'Media T50%': np.mean(data['T50']) if data['T50'] else np.nan,
            'Media T75%': np.mean(data['T75']) if data['T75'] else np.nan,
            'Media T90%': np.mean(data['T90']) if data['T90'] else np.nan,
            'Media T100%': np.mean(data['T100']) if data['T100'] else np.nan,
        }
    
    # Crear y mostrar el DataFrame
    df = pd.DataFrame.from_dict(summary_data, orient='index')
    df.index.name = "Casuística de Escenario"
    
    # display() imprimirá una tabla HTML bonita en notebooks,
    # o un print() normal en la terminal
    display(df.round(2))
    print("\n" + "="*50)


    # --- 4. Mostrar Gráficos Comparativos ---
    
    titles = list(all_run_data.keys())
    if len(titles) != 2:
        print("Error: Se necesitan exactamente dos casuísticas para la comparación gráfica.")
        return
        
    # --- Gráfico 1: Histograma Comparativo (Lado a Lado) ---
    
    fig_hist, (ax1_hist, ax2_hist) = plt.subplots(1, 2, figsize=(16, 6), sharey=True)
    
    # Histograma Casuística A
    times_a = all_individual_times[titles[0]]
    if times_a:
        mean_time_a = np.mean(times_a)
        median_time_a = np.median(times_a)
        ax1_hist.hist(times_a, bins=50, alpha=0.75, edgecolor='black', color='#0077BE')
        ax1_hist.axvline(mean_time_a, color='red', linestyle='dashed', linewidth=2, label=f'Media: {mean_time_a:.2f}s')
        ax1_hist.axvline(median_time_a, color='blue', linestyle='dashed', linewidth=2, label=f'Mediana: {median_time_a:.2f}s')
        ax1_hist.set_title(titles[0])
        ax1_hist.set_xlabel('Tiempo de Evacuación (s)')
        ax1_hist.set_ylabel('Frecuencia (Agentes)')
        ax1_hist.legend()
        ax1_hist.grid(True, axis='y', linestyle='--', alpha=0.5)

    # Histograma Casuística B
    times_b = all_individual_times[titles[1]]
    if times_b:
        mean_time_b = np.mean(times_b)
        median_time_b = np.median(times_b)
        ax2_hist.hist(times_b, bins=50, alpha=0.75, edgecolor='black', color='#D55E00')
        ax2_hist.axvline(mean_time_b, color='red', linestyle='dashed', linewidth=2, label=f'Media: {mean_time_b:.2f}s')
        ax2_hist.axvline(median_time_b, color='blue', linestyle='dashed', linewidth=2, label=f'Mediana: {median_time_b:.2f}s')
        ax2_hist.set_title(titles[1])
        ax2_hist.set_xlabel('Tiempo de Evacuación (s)')
        ax2_hist.legend()
        ax2_hist.grid(True, axis='y', linestyle='--', alpha=0.5)

    fig_hist.suptitle(f'Comparativa de Distribución de Tiempos Individuales (Escenario {main_scenario_id}, n={n_runs})')

    # --- Gráfico 2: Boxplot Comparativo (Lado a Lado) ---

    fig_box, (ax1_box, ax2_box) = plt.subplots(1, 2, figsize=(16, 7), sharey=True)

    # Función helper interna para crear un boxplot
    def plot_boxplot(ax, title, run_data):
        data_to_plot = []
        labels = []
        
        # Añadir datos solo si existen (para evitar errores con TIMEOUTS)
        if run_data['T50']:
            data_to_plot.append(run_data['T50'])
            labels.append(f"T 50%\n(Mediana: {np.median(run_data['T50']):.2f}s)")
        if run_data['T75']:
            data_to_plot.append(run_data['T75'])
            labels.append(f"T 75%\n(Mediana: {np.median(run_data['T75']):.2f}s)")
        if run_data['T90']:
            data_to_plot.append(run_data['T90'])
            labels.append(f"T 90%\n(Mediana: {np.median(run_data['T90']):.2f}s)")
        if run_data['T100']:
            data_to_plot.append(run_data['T100'])
            labels.append(f"T 100%\n(Mediana: {np.median(run_data['T100']):.2f}s)")

        if data_to_plot:
            ax.boxplot(data_to_plot, labels=labels)
            ax.set_title(title)
            ax.set_ylabel('Tiempo (s)')
            ax.grid(True, axis='y', linestyle='--', alpha=0.5)
        else:
            ax.text(0.5, 0.5, "Sin datos para mostrar", 
                    horizontalalignment='center', verticalalignment='center', 
                    transform=ax.transAxes)
    
    # Crear los boxplots
    plot_boxplot(ax1_box, titles[0], all_run_data[titles[0]])
    plot_boxplot(ax2_box, titles[1], all_run_data[titles[1]])
    
    fig_box.suptitle(f'Comparativa de Tiempos de Evacuación por Percentil (Escenario {main_scenario_id}, n={n_runs})')
    
    plt.show()