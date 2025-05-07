# main.py
# Hauptskript zur Steuerung von zwei TMC2209 Motoren über separate UART-Schnittstellen.
# Motoren laufen sequenziell (nacheinander).

from tmc_driver import TMC_2209, Loglevel, MovementAbsRel, Direction # Angepasste Bibliothek importieren
import time
import random
import _thread # Wird hier nicht mehr für Motorsteuerung verwendet, aber gc etc. sind noch da
import gc
import machine

print("---")
print("SKRIPT START")
print("---")

# --- Globale Konfigurationen ---
PIN_EN = 26  # Gemeinsamer Enable-Pin für beide Treiber (Low-Aktiv)

# UART und Treiber Pin-Definitionen
# Treiber 1
UART0_ID = 0
UART0_TX_PIN = 0  # GP0 als UART0 TX
UART0_RX_PIN = 1  # GP1 als UART0 RX
TMC1_STEP_PIN = 27
TMC1_DIR_PIN = 14
TMC1_SERIAL_ADDRESS = 0 # UART Adresse des TMC2209 Chips

# Treiber 2
UART1_ID = 1
UART1_TX_PIN = 4  # GP4 als UART1 TX
UART1_RX_PIN = 5  # GP5 als UART1 RX
TMC2_STEP_PIN = 28
TMC2_DIR_PIN = 15
TMC2_SERIAL_ADDRESS = 0 # UART Adresse des TMC2209 Chips

# Allgemeine Motoreinstellungen
MOTOR_BAUDRATE = 115200
MOTOR_STEPS_PER_REV = 200
MOTOR_MICROSTEPS = 256
MOTOR_RUN_CURRENT_MA = 350
MOTOR_ACCELERATION = 200
MOTOR_MAX_SPEED = 400
NUM_MOVEMENTS_PER_TURN = 5 # Anzahl der Zufallsbewegungen pro Motor, bevor gewechselt wird

# Globale Variable für den Fehlerstatus
critical_error_occurred = False

# --- Initialisierung der Treiber ---
tmc1 = None
tmc2 = None

try:
    print("Initialisiere TMC1...")
    tmc1 = TMC_2209(step_pin=TMC1_STEP_PIN, dir_pin=TMC1_DIR_PIN, en_pin=PIN_EN,
                     uart_id=UART0_ID, tx_pin_gp_num=UART0_TX_PIN, rx_pin_gp_num=UART0_RX_PIN,
                     tmc_serial_address=TMC1_SERIAL_ADDRESS, baudrate=MOTOR_BAUDRATE,
                     steps_per_rev_motor=MOTOR_STEPS_PER_REV)
    print("TMC1 initialisiert.")

    print("Initialisiere TMC2...")
    tmc2 = TMC_2209(step_pin=TMC2_STEP_PIN, dir_pin=TMC2_DIR_PIN, en_pin=PIN_EN,
                     uart_id=UART1_ID, tx_pin_gp_num=UART1_TX_PIN, rx_pin_gp_num=UART1_RX_PIN,
                     tmc_serial_address=TMC2_SERIAL_ADDRESS, baudrate=MOTOR_BAUDRATE,
                     steps_per_rev_motor=MOTOR_STEPS_PER_REV)
    print("TMC2 initialisiert.")

except Exception as e:
    print(f"FEHLER bei der Initialisierung der TMC-Treiber: {e}")
    critical_error_occurred = True

if not critical_error_occurred:
    # --- Konfiguration der Treiber ---
    drivers = {"TMC1": tmc1, "TMC2": tmc2}
    for name, tmc_driver in drivers.items():
        if tmc_driver:
            print(f"Konfiguriere {name}...")
            tmc_driver.setLoglevel(Loglevel.info)
            tmc_driver.setMovementAbsRel(MovementAbsRel.relative)
            tmc_driver.setDirection_reg(False)
            tmc_driver.setVSense(True)
            tmc_driver.setInternalRSense(False)
            tmc_driver.setInterpolation(True)
            tmc_driver.setSpreadCycle(False)
            tmc_driver.setMicrosteppingResolution(MOTOR_MICROSTEPS)
            tmc_driver.setCurrent(MOTOR_RUN_CURRENT_MA, hold_current_multiplier=0.7, ihold_delay=8)
            tmc_driver.setAcceleration(MOTOR_ACCELERATION)
            tmc_driver.setMaxSpeed(MOTOR_MAX_SPEED)
            # tmc_driver.readDRVSTATUS() # Zum Testen Status lesen
    print("TMC-Registereinstellungen vorgenommen.")
    print("---\n---")

    # --- Motoren aktivieren ---
    try:
        if tmc1: tmc1.setMotorEnabled(True)
        if tmc2: tmc2.setMotorEnabled(True)
        print("Motoren aktiviert.")
    except Exception as e:
        print(f"FEHLER beim Aktivieren der Motoren: {e}")
        critical_error_occurred = True

# --- Bewegungsbefehle aus Datei lesen ---
movement_commands = []
if not critical_error_occurred:
    try:
        with open('commands.txt', 'r') as file:
            for line in file:
                stripped_line = line.strip()
                if stripped_line:
                    try:
                        movement_commands.append(int(stripped_line))
                    except ValueError:
                        print(f"WARNUNG: Ungültige Zeile in commands.txt übersprungen: {stripped_line}")
        if not movement_commands:
            print("WARNUNG: commands.txt ist leer oder enthält keine gültigen Zahlen.")
        else:
            print(f"Bewegungsbefehle geladen: {movement_commands}")
    except OSError:
        print("FEHLER: Datei 'commands.txt' nicht gefunden.")
    except Exception as e:
        print(f"FEHLER beim Lesen von 'commands.txt': {e}")

print("---\n---")

# --- Modifizierte Bewegungsfunktion ---
def execute_motor_movements(motor_controller, motor_id, commands, num_moves):
    """Führt eine definierte Anzahl von Zufallsbewegungen für einen Motor aus."""
    global critical_error_occurred
    if not commands or motor_controller is None:
        print(f"Motor {motor_id}: Keine Befehle oder Treiber nicht initialisiert.")
        return

    if motor_controller._loglevel >= Loglevel.debug:
        print(f"Motor {motor_id}: Führt {num_moves} Bewegungen aus.")

    for i in range(num_moves):
        if critical_error_occurred:
            print(f"Motor {motor_id}: Kritischer Fehler aufgetreten, breche Bewegungen ab.")
            break
        try:
            if not movement_commands:
                print(f"Motor {motor_id}: Keine Befehle mehr.")
                break

            command = random.choice(commands)
            
            if abs(command) > 0:
                if motor_controller._loglevel >= Loglevel.debug:
                    print(f"Motor {motor_id} (Bewegung {i+1}/{num_moves}): Akt.Pos: {motor_controller.getCurrentPosition()}, Bewege um {command} Schritte.")
                
                motor_controller.runToPositionSteps(command, MovementAbsRel.relative)
                
                if motor_controller._loglevel >= Loglevel.debug:
                    print(f"Motor {motor_id}: Bewegung beendet. Neue Pos: {motor_controller.getCurrentPosition()}")
            
            # Kurze Pause nach jeder einzelnen Bewegung innerhalb des "Turns" eines Motors
            time.sleep_ms(random.randint(20, 100)) 

        except Exception as e:
            print(f"FEHLER im Motor {motor_id} während Bewegung {i+1}: {e}")
            # critical_error_occurred = True # Bei schwerwiegenden Fehlern hier setzen
            time.sleep_ms(500) # Längere Pause nach einem Fehler
            break # Breche die Bewegungsschleife für diesen Motor ab
    
    if motor_controller._loglevel >= Loglevel.debug:
        print(f"Motor {motor_id}: {num_moves} Bewegungen ausgeführt.")


# --- Hauptlogik für sequenziellen Betrieb ---
if not critical_error_occurred and movement_commands:
    main_loop_count = 0
    try:
        print("Starte sequenziellen Motorbetrieb...")
        while not critical_error_occurred: # Hauptschleife für abwechselnden Betrieb
            main_loop_count += 1
            print(f"\n--- Hauptschleife Durchlauf: {main_loop_count} ---")

            if tmc1:
                print(f"--- Motor 1 (Main) ist dran ---")
                execute_motor_movements(tmc1, "1 (Main)", movement_commands, NUM_MOVEMENTS_PER_TURN)
                if critical_error_occurred: break # Überprüfe nach jedem Motorblock
            
            # Pause zwischen den Motor-Turns
            time.sleep_ms(random.randint(100, 300)) 
            gc.collect() # Garbage Collection zwischen den Motor-Turns

            if tmc2:
                print(f"--- Motor 2 (Sequenziell) ist dran ---")
                execute_motor_movements(tmc2, "2 (Seq)", movement_commands, NUM_MOVEMENTS_PER_TURN)
                if critical_error_occurred: break # Überprüfe nach jedem Motorblock

            # Pause bevor die Hauptschleife von vorne beginnt
            time.sleep_ms(random.randint(200, 500))
            print(f"Freier Speicher nach Durchlauf {main_loop_count}: {gc.mem_free()} Bytes")

    except KeyboardInterrupt:
        print("\nProgramm durch Benutzer unterbrochen (KeyboardInterrupt).")
        critical_error_occurred = True
    except Exception as e:
        print(f"FEHLER in der Hauptschleife des sequenziellen Betriebs: {e}")
        critical_error_occurred = True
else:
    if critical_error_occurred:
        print("Programm startet nicht aufgrund eines kritischen Initialisierungsfehlers.")
    elif not movement_commands:
        print("Keine Bewegungsbefehle vorhanden. Motoren werden nicht bewegt.")
        time.sleep(5)


# --- Aufräumen ---
print("\n---")
print("Deaktiviere Motoren und räume auf...")
try:
    if tmc1:
        tmc1.stop()
        tmc1.setMotorEnabled(False)
    if tmc2:
        tmc2.stop()
        tmc2.setMotorEnabled(False)
    print("Motoren deaktiviert.")
except Exception as e:
    print(f"FEHLER beim Deaktivieren der Motoren: {e}")

gc.collect()
print("---")
print("SKRIPT BEENDET")
print(f"Freier Speicher am Ende: {gc.mem_free()} Bytes")
print("---")
