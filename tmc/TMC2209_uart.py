# tmc_driver.py
# Angepasste Bibliothek für TMC2209 Schrittmotortreiber zur Nutzung mit separaten UART-Schnittstellen am Raspberry Pi Pico.

import machine
import time
import math
import struct

# -----------------------------------------------------------------------
# 1. Registerdefinitionen (TMC_2209_reg)
# -----------------------------------------------------------------------
class TMC_2209_reg:
    GCONF           = 0x00  # Global configuration flags
    GSTAT           = 0x01  # Global status flags
    IFCNT           = 0x02  # Interface transmission counter.
    IOIN            = 0x06  # Reads the state of all TMC2209 pins
    IHOLD_IRUN      = 0x10  # Driver current control
    TSTEP           = 0x12  # Actual measured motor speed
    TCOOLTHRS       = 0x14  # Upper velocity limit for StealthChopTM operation
    SGTHRS          = 0x40  # StallGuard4TM threshold value
    SG_RESULT       = 0x41  # StallGuard4TM result
    MSCNT           = 0x6A  # Microstep counter
    CHOPCONF        = 0x6C  # Chopper and driver configuration
    DRVSTATUS       = 0x6F  # Driver status flags and current level

    # GCONF Bits
    i_scale_analog      = 1 << 0
    internal_rsense     = 1 << 1  # Use internal R_sense. External resistors are disabled.
    en_spreadcycle      = 1 << 2  # Enable SpreadCycle mode
    shaft               = 1 << 3  # Inverse motor direction
    index_otpw          = 1 << 4  # INDEX pin outputs overtemperature prewarning flag
    index_step          = 1 << 5  # INDEX output shows step pulses
    mstep_reg_select    = 1 << 7  # Microstep resolution selected by MSTEP register

    # GSTAT Bits
    reset               = 1 << 0  # Indicates that the IC has been reset
    drv_err             = 1 << 1  # Indicates that the driver has been shut down
    uv_cp               = 1 << 2  # Indicates an undervoltage on the charge pump

    # CHOPCONF Bits
    vsense              = 1 << 17 # Sense resistor voltage select for current scaling
    msres0              = 1 << 24 # Microstep resolution bits
    msres1              = 1 << 25
    msres2              = 1 << 26
    msres3              = 1 << 27
    intpol              = 1 << 28 # Enable interpolation to 256 microsteps

    # IOIN Bits (actual pin states)
    io_enn              = 1 << 0  # ENN pin
    io_step             = 1 << 7  # STEP pin
    io_spread           = 1 << 8  # SPREAD pin (TMC2209: PDN_UART)
    io_dir              = 1 << 9  # DIR pin

    # DRVSTATUS Bits
    stst                = 1 << 31 # Standstill indicator
    stealth             = 1 << 30 # StealthChop indicator
    cs_actual_mask      = 0x1F0000 # Mask for CS_ACTUAL [20:16]
    cs_actual_shift     = 16
    olb                 = 1 << 7  # Open load indicator bridge B
    ola                 = 1 << 6  # Open load indicator bridge A
    s2vsb               = 1 << 5  # Short to supply indicator phase B
    s2vsa               = 1 << 4  # Short to supply indicator phase A
    s2gb                = 1 << 3  # Short to ground indicator phase B
    s2ga                = 1 << 2  # Short to ground indicator phase A
    ot                  = 1 << 1  # Over temperature flag
    otpw                = 1 << 0  # Over temperature pre-warning flag

    # IHOLD_IRUN Bits
    ihold_mask          = 0x1F      # Mask for IHOLD [4:0]
    ihold_shift         = 0
    irun_mask           = 0x1F00    # Mask for IRUN [12:8]
    irun_shift          = 8
    iholddelay_mask     = 0xF0000   # Mask for IHOLDDELAY [19:16]
    iholddelay_shift    = 16

    # SGTHRS Bits
    sgthrs_mask         = 0xFF      # Mask for SGTHRS [7:0]

    # Microstep Resolution Values (for MRES field in CHOPCONF)
    # Value to write to MRES field for 2^(8-value) microsteps
    # Example: for 256 (2^8) microsteps, MRES_val = 0 (8-8=0)
    # For 1 (2^0) microstep (full step), MRES_val = 8 (8-0=8)
    # The library's setMicrosteppingResolution handles this logic.
    # These are the values the CHOPCONF.MRES field would take.
    _mres_map = {
        256: 0, 128: 1, 64: 2, 32: 3, 16: 4, 8: 5, 4: 6, 2: 7, 1: 8
    }

# -----------------------------------------------------------------------
# 2. UART Kommunikation (TMC_UART)
# -----------------------------------------------------------------------
class TMC_UART:
    def __init__(self, uart_id, baudrate, tx_pin_gp_num, rx_pin_gp_num, tmc_serial_address=0):
        """
        Initialisiert die UART-Kommunikation mit einem TMC-Treiber.
        :param uart_id: 0 oder 1 für die Hardware-UART-Instanz des Pico.
        :param baudrate: Baudrate, z.B. 115200.
        :param tx_pin_gp_num: GPIO-Nummer des TX-Pins für diese UART-Instanz.
        :param rx_pin_gp_num: GPIO-Nummer des RX-Pins für diese UART-Instanz.
        :param tmc_serial_address: Die Slave-Adresse des TMC-Treibers (0-3).
        """
        self.tx_pin = machine.Pin(tx_pin_gp_num)
        self.rx_pin = machine.Pin(rx_pin_gp_num)
        self.ser = machine.UART(uart_id, baudrate=baudrate, tx=self.tx_pin, rx=self.rx_pin)
        self.ser.init(baudrate, bits=8, parity=None, stop=1, timeout=10, timeout_char=10) # timeout in ms

        self.tmc_slave_address = tmc_serial_address # Adresse des TMC-Chips (0-3)
        
        # Pause für die Kommunikation, abhängig von der Baudrate.
        # Eine zu kurze Pause kann zu Lesefehlern führen.
        # 10 Bytes (max frame size) * 10 bits/byte (8 data + start + stop) / baudrate
        self.communication_pause_us = int((10 * 10 / baudrate) * 1000000 * 1.5) # *1.5 als Sicherheitsfaktor
        if self.communication_pause_us < 200: # Mindestpause
             self.communication_pause_us = 200


    def _compute_crc8_atm(self, datagram, initial_value=0):
        crc = initial_value
        for byte in datagram:
            for _ in range(8):
                if (crc >> 7) ^ (byte & 0x01):
                    crc = ((crc << 1) ^ 0x07) & 0xFF
                else:
                    crc = (crc << 1) & 0xFF
                byte = byte >> 1
        return crc

    def read_reg(self, reg_addr):
        """Liest einen 32-Bit-Wert aus einem Register."""
        # Aufbau des Read-Request-Datagramms: Sync, SlaveAddr, RegAddr, CRC
        # Sync = 0x05 (konstant)
        datagram = bytearray(4)
        datagram[0] = 0x05  # Sync-Nibble + Master Address (immer 0 für Master)
        datagram[1] = self.tmc_slave_address # Slave-Adresse des TMC-Chips
        datagram[2] = reg_addr & 0x7F # Registeradresse (MSB=0 für Lesen)
        datagram[3] = self._compute_crc8_atm(datagram[:-1])

        self.ser.flush() # Alten Puffer leeren
        self.ser.write(datagram)
        
        time.sleep_us(self.communication_pause_us) # Warten auf Antwort

        response = self.ser.read(8) # Antwort ist 8 Bytes lang: Sync, MasterAddr, RegAddr, 4 Datenbytes, CRC

        if response is None or len(response) < 8:
            # print(f"TMC_UART Read Error: Keine oder unvollständige Antwort für Register 0x{reg_addr:02X}. Erhalten: {response}")
            return None # Fehler oder Timeout

        # CRC der Antwort prüfen
        # Antwort-Datagramm für CRC-Prüfung: Sync, MasterAddr, RegAddr, D3, D2, D1, D0
        # Wichtig: Die Reihenfolge der Datenbytes im CRC ist die gleiche wie im Frame, nicht invertiert.
        crc_payload = response[:-1] # Alles außer dem empfangenen CRC
        calculated_crc = self._compute_crc8_atm(crc_payload)
        received_crc = response[7]

        if calculated_crc != received_crc:
            # print(f"TMC_UART Read CRC Error: Register 0x{reg_addr:02X}. Erw: {calculated_crc:02X}, Empf: {received_crc:02X}. Antwort: {response}")
            return None

        # Datenbytes extrahieren (Big Endian) und zu Integer konvertieren
        # Antwort: Sync, MasterAddr, RegAddr, D3(MSB), D2, D1, D0(LSB), CRC
        value = struct.unpack('>I', response[3:7])[0]
        return value

    def write_reg(self, reg_addr, value):
        """Schreibt einen 32-Bit-Wert in ein Register."""
        # Aufbau des Write-Datagramms: Sync, SlaveAddr, RegAddr+WriteBit, D3, D2, D1, D0, CRC
        datagram = bytearray(8)
        datagram[0] = 0x05  # Sync-Nibble
        datagram[1] = self.tmc_slave_address
        datagram[2] = (reg_addr & 0x7F) | 0x80 # Registeradresse mit Write-Bit (MSB=1)
        
        # Wert in 4 Bytes aufteilen (Big Endian)
        datagram[3] = (value >> 24) & 0xFF # D3 (MSB)
        datagram[4] = (value >> 16) & 0xFF # D2
        datagram[5] = (value >> 8) & 0xFF  # D1
        datagram[6] = value & 0xFF         # D0 (LSB)
        
        datagram[7] = self._compute_crc8_atm(datagram[:-1])

        self.ser.flush()
        self.ser.write(datagram)
        time.sleep_us(self.communication_pause_us) # Kurze Pause nach dem Schreiben
        return True # Einfache Bestätigung, tatsächliche Überprüfung wäre komplexer

    def flush_serial_buffer(self):
        self.ser.flush()

    def set_bit(self, value, bit_mask):
        return value | bit_mask

    def clear_bit(self, value, bit_mask):
        return value & ~bit_mask

    def __del__(self):
        if self.ser:
            self.ser.deinit()


# -----------------------------------------------------------------------
# 3. Hauptklasse für den TMC2209 Treiber (TMC_2209_StepperDriver)
# -----------------------------------------------------------------------
class Direction():
    CCW = 0  # Counter Clockwise
    CW  = 1  # Clockwise

class Loglevel():
    none     = 0
    error    = 10
    info     = 20
    debug    = 30
    movement = 40
    all      = 100

class MovementAbsRel():
    absolute = 0
    relative = 1

class TMC_2209:
    def __init__(self, step_pin, dir_pin, en_pin,
                 uart_id, tx_pin_gp_num, rx_pin_gp_num, tmc_serial_address=0,
                 baudrate=115200, steps_per_rev_motor=200):
        
        self.reg = TMC_2209_reg() # Zugriff auf Registerdefinitionen
        self.tmc_uart = TMC_UART(uart_id, baudrate, tx_pin_gp_num, rx_pin_gp_num, tmc_serial_address)
        
        self._pin_step_num = step_pin
        self._pin_dir_num = dir_pin
        self._pin_en_num = en_pin
        
        self.p_pin_step = machine.Pin(self._pin_step_num, machine.Pin.OUT)
        self.p_pin_dir = machine.Pin(self._pin_dir_num, machine.Pin.OUT)
        self.p_pin_en = machine.Pin(self._pin_en_num, machine.Pin.OUT)
        
        self._direction_pin_state = Direction.CW # Annahme: HIGH = CW, anpassbar
        self.p_pin_dir.value(self._direction_pin_state)
        self.setMotorEnabled(False) # Motor initial deaktiviert

        self._loglevel = Loglevel.info
        self._movement_abs_rel = MovementAbsRel.absolute
        
        self._motor_steps_per_revolution = steps_per_rev_motor # z.B. 200 für 1.8° Motor
        self._current_microsteps = self.getMicrosteppingResolution(from_hw=True) # Lese aktuelle Mikroschritte
        if self._current_microsteps is None: # Falls Lesen fehlschlägt
            print(f"WARNUNG: Konnte Mikroschritte nicht von TMC an UART {uart_id} lesen. Setze auf 256.")
            self._current_microsteps = 256 # Fallback
            # Versuche, Standardwerte zu setzen, falls Kommunikation initial nicht klappt
            self.setMicrosteppingResolution(256) # Setze auf 256 als Standard
            self.setInterpolation(True)

        self._steps_per_revolution_actual = self._motor_steps_per_revolution * self._current_microsteps

        # Bewegungsparameter
        self._currentPos = 0
        self._targetPos = 0
        self._speed = 0.0
        self._maxSpeed = 200.0 # steps/sec
        self._acceleration = 200.0 # steps/sec^2
        self._stepInterval = 0
        self._lastStepTime = 0
        self._n = 0 # Schrittzähler für Beschleunigungsprofil
        self._c0 = 0 # Initialer Schrittintervall für Beschleunigung
        self._cn = 0 # Aktueller Schrittintervall
        self._cmin = 1000000.0 / self._maxSpeed # Minimaler Schrittintervall
        self._stop_requested = False

        if self._loglevel >= Loglevel.info:
            print(f"TMC2209 (UART{uart_id}, Addr {tmc_serial_address}): Init. Steps/Rev: {self._steps_per_revolution_actual} (MSteps: {self._current_microsteps})")
        
        self.clearGSTAT() # Statusregister löschen

    def _read_reg(self, reg_addr):
        val = self.tmc_uart.read_reg(reg_addr)
        if val is None and self._loglevel >= Loglevel.error:
            print(f"TMC2209 Error: Konnte Register 0x{reg_addr:02X} nicht lesen.")
        return val

    def _write_reg(self, reg_addr, value):
        if self._loglevel >= Loglevel.debug:
            print(f"TMC2209 Write: Reg 0x{reg_addr:02X} = 0x{value:08X}")
        return self.tmc_uart.write_reg(reg_addr, value)

    def setLoglevel(self, loglevel):
        self._loglevel = loglevel

    def setMovementAbsRel(self, movement_abs_rel):
        self._movement_abs_rel = movement_abs_rel

    def setMotorEnabled(self, en):
        # EN-Pin ist oft Low-Aktiv
        self.p_pin_en.value(not en)
        if self._loglevel >= Loglevel.info:
            print(f"TMC2209: Motor Aktiviert: {en}")

    def setDirection_pin(self, direction_cw): # True für CW, False für CCW
        self._direction_pin_state = bool(direction_cw)
        self.p_pin_dir.value(self._direction_pin_state)

    # --- Registerbasierte Konfigurationen ---
    def clearGSTAT(self):
        gstat = self._read_reg(self.reg.GSTAT)
        if gstat is not None:
            # GSTAT Bits werden durch Schreiben einer '1' gelöscht
            self._write_reg(self.reg.GSTAT, gstat | self.reg.reset | self.reg.drv_err | self.reg.uv_cp)

    def setDirection_reg(self, reverse_direction): # True invertiert die Richtung im Register
        gconf = self._read_reg(self.reg.GCONF)
        if gconf is not None:
            if reverse_direction:
                gconf = self.tmc_uart.set_bit(gconf, self.reg.shaft)
            else:
                gconf = self.tmc_uart.clear_bit(gconf, self.reg.shaft)
            self._write_reg(self.reg.GCONF, gconf)

    def setIScaleAnalog(self, enable): # True: VREF als Stromreferenz
        gconf = self._read_reg(self.reg.GCONF)
        if gconf is not None:
            if enable:
                gconf = self.tmc_uart.set_bit(gconf, self.reg.i_scale_analog)
            else:
                gconf = self.tmc_uart.clear_bit(gconf, self.reg.i_scale_analog)
            self._write_reg(self.reg.GCONF, gconf)
            
    def setInternalRSense(self, enable): # True: Interne Rsense (ACHTUNG, oft nicht empfohlen)
        gconf = self._read_reg(self.reg.GCONF)
        if gconf is not None:
            if enable:
                gconf = self.tmc_uart.set_bit(gconf, self.reg.internal_rsense)
            else:
                gconf = self.tmc_uart.clear_bit(gconf, self.reg.internal_rsense)
            self._write_reg(self.reg.GCONF, gconf)

    def setSpreadCycle(self, enable_spread): # True: SpreadCycle, False: StealthChop
        gconf = self._read_reg(self.reg.GCONF)
        if gconf is not None:
            if enable_spread:
                gconf = self.tmc_uart.set_bit(gconf, self.reg.en_spreadcycle)
            else:
                gconf = self.tmc_uart.clear_bit(gconf, self.reg.en_spreadcycle)
            self._write_reg(self.reg.GCONF, gconf)
            if self._loglevel >= Loglevel.info:
                print(f"TMC2209: SpreadCycle {'aktiviert' if enable_spread else 'deaktiviert (StealthChop aktiv)'}")

    def setInterpolation(self, enable): # True: Interpolation auf 256 Mikroschritte
        chopconf = self._read_reg(self.reg.CHOPCONF)
        if chopconf is not None:
            if enable:
                chopconf = self.tmc_uart.set_bit(chopconf, self.reg.intpol)
            else:
                chopconf = self.tmc_uart.clear_bit(chopconf, self.reg.intpol)
            self._write_reg(self.reg.CHOPCONF, chopconf)

    def setMicrosteppingResolution(self, mres_value): # z.B. 1, 2, 8, 16, 256
        if mres_value not in self.reg._mres_map:
            if self._loglevel >= Loglevel.error:
                print(f"TMC2209 Error: Ungültige Mikroschrittauflösung: {mres_value}")
            return

        mres_reg_val = self.reg._mres_map[mres_value]
        
        chopconf = self._read_reg(self.reg.CHOPCONF)
        if chopconf is not None:
            # Bestehende MRES Bits löschen (Maske: ~(0b1111 << 24))
            chopconf &= ~(0xF << 24)
            # Neue MRES Bits setzen
            chopconf |= (mres_reg_val << 24)
            self._write_reg(self.reg.CHOPCONF, chopconf)
            self._current_microsteps = mres_value
            self._steps_per_revolution_actual = self._motor_steps_per_revolution * self._current_microsteps
            if self._loglevel >= Loglevel.info:
                 print(f"TMC2209: Mikroschrittauflösung gesetzt auf 1/{mres_value}. Steps/Rev: {self._steps_per_revolution_actual}")

            # Sicherstellen, dass MSTEP über Register ausgewählt wird
            gconf = self._read_reg(self.reg.GCONF)
            if gconf is not None and not (gconf & self.reg.mstep_reg_select):
                gconf = self.tmc_uart.set_bit(gconf, self.reg.mstep_reg_select)
                self._write_reg(self.reg.GCONF, gconf)
    
    def getMicrosteppingResolution(self, from_hw=False):
        if not from_hw:
            return self._current_microsteps
        
        chopconf = self._read_reg(self.reg.CHOPCONF)
        if chopconf is None: return None # Fehler beim Lesen
        
        mres_reg_val_read = (chopconf >> 24) & 0xF
        for steps, val_in_map in self.reg._mres_map.items():
            if val_in_map == mres_reg_val_read:
                self._current_microsteps = steps
                return steps
        return None # Unbekannter Wert gelesen

    def setVSense(self, high_sensitivity): # True: hohe Empfindlichkeit (VSENSE=1)
        chopconf = self._read_reg(self.reg.CHOPCONF)
        if chopconf is not None:
            if high_sensitivity:
                chopconf = self.tmc_uart.set_bit(chopconf, self.reg.vsense)
            else:
                chopconf = self.tmc_uart.clear_bit(chopconf, self.reg.vsense)
            self._write_reg(self.reg.CHOPCONF, chopconf)

    def setCurrent(self, run_current_ma, hold_current_multiplier=0.5, ihold_delay=4, rsense=0.11):
        # Formel aus dem TMC2209 Datenblatt (vereinfacht)
        # CS = (I_rms * 2.5 * (Rsense + 0.02)) / (Vfs * sqrt(2)) - 1
        # Vfs ist abhängig von VSENSE. Annahme: VREF = 3.3V oder interner Referenz
        # VSENSE = 0 (low sensitivity) -> VFS approx 0.325V
        # VSENSE = 1 (high sensitivity) -> VFS approx 0.180V
        
        # Lese VSENSE Bit
        chopconf = self._read_reg(self.reg.CHOPCONF)
        if chopconf is None:
            if self._loglevel >= Loglevel.error:
                print("TMC2209 Error: setCurrent - CHOPCONF konnte nicht gelesen werden.")
            return

        vsense_is_high_sensitivity = bool(chopconf & self.reg.vsense)
        
        if vsense_is_high_sensitivity: # VSENSE = 1
            vfs = 0.180 
        else: # VSENSE = 0
            vfs = 0.325
        
        # IRUN (max 31)
        irun_cs = int(round(( (run_current_ma / 1000.0) * (rsense + 0.020) * 255.0 ) / (vfs * 32.0 * 1.41421) -1 )) # Grobe Näherung
        irun_cs = max(0, min(31, irun_cs))

        # IHOLD (max 31)
        ihold_cs = int(round(irun_cs * hold_current_multiplier))
        ihold_cs = max(0, min(31, ihold_cs))

        ihold_delay_val = max(0, min(15, int(ihold_delay)))

        ihold_irun_val = (ihold_cs << self.reg.ihold_shift) | \
                           (irun_cs << self.reg.irun_shift) | \
                           (ihold_delay_val << self.reg.iholddelay_shift)
        
        self._write_reg(self.reg.IHOLD_IRUN, ihold_irun_val)
        if self._loglevel >= Loglevel.info:
            print(f"TMC2209: Strom gesetzt: IRUN_CS={irun_cs} ({run_current_ma}mA), IHOLD_CS={ihold_cs}, Delay={ihold_delay_val}")

    # --- Bewegungssteuerung ---
    def setMaxSpeed(self, speed_sps): # Schritte pro Sekunde
        if speed_sps <= 0: return
        self._maxSpeed = float(speed_sps)
        self._cmin = 1000000.0 / self._maxSpeed
        if self._n > 0: # Wenn bereits in Bewegung/Beschleunigung
            self._n = (self._speed * self._speed) / (2.0 * self._acceleration)
            self._computeNewSpeed()

    def setAcceleration(self, accel_sps2): # Schritte pro Sekunde^2
        if accel_sps2 <= 0: return
        if self._acceleration == accel_sps2: return
        
        if self._n != 0 : # Neuberechnung von n, falls bereits in Bewegung
             self._n = self._n * (self._acceleration / accel_sps2)
        
        # c0 ist der initiale Schrittintervall für den ersten Schritt aus dem Stillstand
        self._c0 = 0.676 * math.sqrt(2.0 / accel_sps2) * 1000000.0 # Gleichung 15 aus AccelStepper
        self._acceleration = float(accel_sps2)
        self._computeNewSpeed() # Neuberechnung, falls sich _c0 geändert hat und wir stehen

    def _makeAStep(self):
        self.p_pin_step.on()
        time.sleep_us(2) # Kurzer Puls (TMC2209 min. 100ns für STEP)
        self.p_pin_step.off()
        time.sleep_us(2) # Erholzeit

    def _distanceToGo(self):
        return self._targetPos - self._currentPos

    def _computeNewSpeed(self):
        distance_to_go = self._distanceToGo()
        steps_to_stop = (self._speed * self._speed) / (2.0 * self._acceleration) # Gleichung 16

        if distance_to_go == 0 and steps_to_stop <= 1: # Ziel erreicht und fast gestoppt
            self._stepInterval = 0
            self._speed = 0.0
            self._n = 0
            return

        if distance_to_go > 0: # Ziel ist in positiver Richtung
            if self._n > 0: # Aktuell am Beschleunigen
                if steps_to_stop >= distance_to_go or self._direction_pin_state == Direction.CCW:
                    self._n = -int(steps_to_stop) # Beginne Verzögerung
            elif self._n < 0: # Aktuell am Verzögern
                if steps_to_stop < distance_to_go and self._direction_pin_state == Direction.CW:
                    self._n = -self._n # Beginne Beschleunigung (Richtungsumkehr im Profil)
        elif distance_to_go < 0: # Ziel ist in negativer Richtung
            if self._n > 0:
                if steps_to_stop >= -distance_to_go or self._direction_pin_state == Direction.CW:
                    self._n = -int(steps_to_stop)
            elif self._n < 0:
                if steps_to_stop < -distance_to_go and self._direction_pin_state == Direction.CCW:
                    self._n = -self._n
        
        if self._n == 0: # Erster Schritt aus dem Stillstand
            self._cn = self._c0
            self.setDirection_pin(Direction.CW if distance_to_go > 0 else Direction.CCW)
        else: # Folgeschritt
            self._cn = self._cn - ((2.0 * self._cn) / ((4.0 * self._n) + 1)) # Gleichung 13
            self._cn = max(self._cn, self._cmin) # Minimalen Intervall nicht unterschreiten
        
        self._n += 1
        self._stepInterval = self._cn
        self._speed = 1000000.0 / self._cn
        if self._direction_pin_state == Direction.CCW:
            self._speed = -self._speed # Geschwindigkeit ist negativ bei CCW

    def _runSpeed(self): # Wird periodisch aufgerufen
        if not self._stepInterval: # Motor steht oder Ziel erreicht
            return False

        current_time_us = time.ticks_us()
        if time.ticks_diff(current_time_us, self._lastStepTime) >= self._stepInterval:
            if self._direction_pin_state == Direction.CW:
                self._currentPos += 1
            else:
                self._currentPos -= 1
            
            self._makeAStep()
            self._lastStepTime = current_time_us
            return True # Ein Schritt wurde gemacht
        return False # Kein Schritt fällig

    def runToPositionSteps(self, target_pos_steps, movement_mode=None):
        # Bestimme, ob absolut oder relativ bewegt wird
        actual_movement_mode = movement_mode if movement_mode is not None else self._movement_abs_rel
        
        if actual_movement_mode == MovementAbsRel.relative:
            self._targetPos = self._currentPos + target_pos_steps
        else: # Absolut
            self._targetPos = target_pos_steps

        self._stop_requested = False
        
        # Wenn wir bereits stehen und das Ziel das aktuelle Pos ist, nichts tun
        if self._speed == 0.0 and self._targetPos == self._currentPos:
            return True

        # Initialisiere Geschwindigkeitsprofil neu, wenn wir stehen
        if self._speed == 0.0:
            self._n = 0
            self._computeNewSpeed() # Berechnet _cn, _stepInterval, _speed für den ersten Schritt

        while not self._stop_requested:
            if self._runSpeed(): # Wenn ein Schritt gemacht wurde
                self._computeNewSpeed() # Berechne Parameter für nächsten Schritt
            
            if self._distanceToGo() == 0 and self._speed == 0.0: # Ziel erreicht und gestoppt
                break
            
            # Hier könnte ein time.sleep_us(1) oder ähnliches stehen, wenn die Schleife zu schnell läuft
            # und andere Tasks blockiert. Aber runSpeed sollte das durch sein Timing regeln.
            # Für kooperatives Multitasking in MicroPython könnte ein yield nötig sein,
            # aber hier ist es blockierend gedacht.

        return not self._stop_requested # True wenn normal beendet, False wenn durch stop()
        
    def stop(self):
        self._stop_requested = True
        # Sanftes Stoppen: Setze Ziel auf aktuelle Position mit aktueller Geschwindigkeit/Beschleunigung
        # Für ein hartes Stoppen: _targetPos = _currentPos und _speed = 0, _n = 0, _stepInterval = 0
        # Hier implementieren wir ein sanfteres Auslaufen, indem wir das Ziel anpassen.
        # Die runToPositionSteps Schleife wird dann natürlich abbremsen.
        # Effektiver ist es, _targetPos so zu setzen, dass es mit der aktuellen _n zum Stillstand kommt.
        # Fürs Erste setzen wir einfach das Flag. Die Bewegung wird in runToPositionSteps gestoppt.

    def getCurrentPosition(self):
        return self._currentPos

    def setCurrentPosition(self, new_pos):
        self._currentPos = new_pos
        self._targetPos = new_pos # Verhindert ungewollte Bewegung
        self._n = 0
        self._speed = 0.0
        self._stepInterval = 0

    def readDRVSTATUS(self):
        drvstatus = self._read_reg(self.reg.DRVSTATUS)
        if drvstatus is None: return None
        if self._loglevel >= Loglevel.info:
            print(f"TMC2209: DRVSTATUS: 0x{drvstatus:08X}")
            if drvstatus & self.reg.otpw: print("  - OTPW: Übertemperatur Vorwarnung")
            if drvstatus & self.reg.ot: print("  - OT: Übertemperatur Fehler!")
            if drvstatus & self.reg.s2ga: print("  - S2GA: Kurzschluss nach GND A!")
            # ... weitere Statusbits hier ausgeben ...
            cs_actual = (drvstatus & self.reg.cs_actual_mask) >> self.reg.cs_actual_shift
            print(f"  - CS_ACTUAL: {cs_actual}")
            if drvstatus & self.reg.stealth: print("  - StealthChop aktiv")
            if drvstatus & self.reg.stst: print("  - Motor steht (Standstill)")
        return drvstatus

    def __del__(self):
        try:
            self.setMotorEnabled(False)
        except Exception:
            pass # Ignoriere Fehler beim Deinitialisieren
        if self._loglevel >= Loglevel.info:
            print(f"TMC2209 (UART an Pins TX:{self._pin_tx_num} RX:{self._pin_rx_num}): Deinit")
