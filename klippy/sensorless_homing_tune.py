# sensorless_homing_tune.py
import logging
import time
from enum import Enum

class HomingState(Enum):
    Homing_premature = 1
    Homing_adequate = 2
    Homing_timeout = 3

class SensorlessHomingTune:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object('gcode')
        self.axis = None
        self.backtrack_amount = config.getfloat('backtrack_amount', 5.0)
        self.backtrack_to_mid = config.getfloat('backtrack_to_mid', 150.0)
        self.timeout = config.getfloat('timeout', 15.0)
        self.highest_successful_threshold = None

        # Register the new G-code command SENSORLESS_TUNE
        self.gcode.register_command("SENSORLESS_TUNE", self.cmd_sensorless_tune, desc="Tune sensorless homing sensitivity")

    def printInfo(self, msg):
        logging.info(msg)
        self.gcode.respond_info(msg)
               
    def cmd_sensorless_tune(self, gcmd):
        self.axis = gcmd.get('AXES').upper()
        if self.axis not in ['X', 'Y']:
            raise self.gcode.error("Invalid AXES parameter, must be 'X' or 'Y'")

        rangeHigh = 255
        rangeLow = 0
        time_min = 1
        time_max = 10
        while rangeHigh > rangeLow:
            threshold = (rangeHigh + rangeLow) // 2
            r = self._try_homing(self.axis, threshold, time_min, time_max)
            if r == HomingState.Homing_premature:  # Value was too high. Reduce Max
                rangeHigh = threshold - 1
                self._backtrack_head(self.backtrack_amount)
            else:  # It was low enough, and we either succeeded or timed out. Let us still try higher values
                if r == HomingState.Homing_timeout:  # Timed out, so we assume we are at the left rail
                    self.gcode.run_script_from_command(f"SET_KINEMATIC_POSITION {self.axis}=0")
                rangeLow = threshold + 1
                self._backtrack_head(self.backtrack_to_mid)
        self.printInfo(f"Highest threshold: {threshold}")
        
        rangeLow = 0
        while rangeHigh > rangeLow:
            threshold = (rangeHigh + rangeLow) // 2
            r = self._try_homing(self.axis, threshold, time_min, time_max)
            if r == HomingState.Homing_timeout:  # Value was too low, need to increase
                rangeLow = threshold + 1
                self.gcode.run_script_from_command(f"SET_KINEMATIC_POSITION {self.axis}=0")
                self._backtrack_head(self.backtrack_amount)
            else:  # It was still high enough, and we either succeeded or were premature. Let us still try lower values
                rangeHigh = threshold -1
                if r == HomingState.Homing_adequate:
                    self._backtrack_head(self.backtrack_to_mid)
                else:
                    self._backtrack_head(self.backtrack_amount)
        self.printInfo(f"Lowest threshold: {threshold}")
		
 
        
        
    
    def _backtrack_head(self, distance):
        self.gcode.run_script_from_command(f"G1 {self.axis}{distance} F3000")
                
    def _try_homing(self, axis, threshold, mintime, maxtime):
        self.printInfo(f"Trying homing with threshold of {threshold}")
        self._set_threshold(threshold)
        start_time = time.monotonic()
        self.gcode.run_script_from_command(f"G2800 {axis}")
        stall_detected, duration = self._poll_for_stall_detection(start_time, maxtime)
        if stall_detected:
            if duration < mintime:
                return HomingState.Homing_premature
            else:
                return HomingState.Homing_adequate
        return HomingState.Homing_timeout

    def _poll_for_stall_detection(self, start_time, timeout):
        while True:
            if (time.monotonic() - start_time) > timeout:
                logging.info(f"{self.axis} axis stall detection timed out")
                self.gcode.respond_info(f"{self.axis} axis stall detection timed out")
                return False, timeout
            if self._is_stall_detected():
                homing_time = time.monotonic() - start_time
                logging.info(f"{self.axis} axis homing time: {homing_time:.2f} seconds")
                self.gcode.respond_info(f"{self.axis} axis homing time: {homing_time:.2f} seconds")
                return True, homing_time
            time.sleep(0.1)

    def _set_threshold(self, threshold):
        self.gcode.run_script_from_command(f"SET_TMC_FIELD STEPPER=stepper_x FIELD=SGTHRS VALUE={threshold}")
        self.gcode.run_script_from_command(f"SET_TMC_FIELD STEPPER=stepper_y FIELD=SGTHRS VALUE={threshold}")

    def _is_stall_detected(self):
        # Perform lookup for toolhead object here
        toolhead = self.printer.lookup_object('toolhead')
        eventtime = self.printer.get_reactor().monotonic()
        # Access kinematics status to determine homing status
        kin_status = toolhead.get_kinematics().get_status(eventtime)
        homed_axes = kin_status.get('homed_axes', [])
        return self.axis.lower() in homed_axes

def load_config(config):
    return SensorlessHomingTune(config)
