
from src.utils.scenarios import create_scenario
from src.utils.simulation_runner import run_preview, run_test

def main():
    print("--- Simulador de Evacuación SFM (v_colisiones_físicas) ---")
    
    # 1. Elegir Modo
    print("\nSeleccione el modo de ejecución:")
    print("  1. Modo 'Preview' (Animación en tiempo real)")
    print("  2. Modo 'Test' (N corridas para estadísticas)")
    mode_choice = input("Opción (1 o 2): ")
    
    # 2. Elegir Escenario
    print("\nSeleccione el escenario a simular:")
    print("--- Escenario 1 ---")
    print("  1. Una Salida Ancha (4m)")
    print(" 12. Dos Salidas Estrechas (2m c/u)")
    print("--- Escenario 2 ---")
    print("  2. Salida Central (4m)")
    print(" 22. Salida en Esquina (4m)")
    print("--- Escenario 3 ---")
    print("  3. Planta Abierta")
    print(" 32. Planta Compartimentada")
    print("--- Escenario 4 (FIS) ---")
    print("  4. Salida Ancha (Sin obstáculo)")
    print(" 42. Salida Ancha (CON obstáculo)")
    
    try:
        scenario_choice = int(input("Opción de escenario (ej. 1, 12, 22...): "))
    except ValueError:
        print("Error: Opción no válida.")
        return

    # 3. Ejecutar
    if mode_choice == '1':
        print("Iniciando Modo 'Preview'...")
        world_obj = create_scenario(scenario_choice)
        if world_obj:
            run_preview(world_obj)
    elif mode_choice == '2':
        try:
            n_runs = int(input("Número de corridas para el test (ej. 20): "))
        except ValueError:
            print("Error: Número no válido. Usando 10 por defecto.")
            n_runs = 10
            
        print("Iniciando Modo 'Test'...")
        run_test(scenario_choice, n_runs)
    else:
        print("Error: Modo no reconocido.")


if __name__ == "__main__":
    main()