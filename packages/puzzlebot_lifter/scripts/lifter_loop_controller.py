#!/usr/bin/env python3
import time
import gpiod

class LifterStatus:
    MOVE_UP = 0
    MOVE_DOWN = 1
    STOP = 2

class LifterControl:
    def __init__(self, chip_name='gpiochip0'):
        self.GPIO_DIR = 149
        self.GPIO_EN = 12
        self.GPIO_BUTTON_UP = 216
        self.GPIO_BUTTON_DOWN = 77

        self.chip = gpiod.Chip(chip_name)
        self.dir_line = self.chip.get_line(self.GPIO_DIR)
        self.en_line = self.chip.get_line(self.GPIO_EN)
        self.up_sensor = self.chip.get_line(self.GPIO_BUTTON_UP)
        self.down_sensor = self.chip.get_line(self.GPIO_BUTTON_DOWN)

        self.dir_line.request(consumer='lifter', type=gpiod.LINE_REQ_DIR_OUT, default_val=0)
        self.en_line.request(consumer='lifter', type=gpiod.LINE_REQ_DIR_OUT, default_val=0)
        self.up_sensor.request(consumer='lifter', type=gpiod.LINE_REQ_DIR_IN)
        self.down_sensor.request(consumer='lifter', type=gpiod.LINE_REQ_DIR_IN)

        self.state = LifterStatus.STOP
        self.previous_state = LifterStatus.STOP
        self.overrun_start_time = None
        self.completed = False

    def read_sensors(self):
        #Order is 0 for up sensor, 1 for down sensor
        # Return True if the sensor is triggered (active low)
        return not self.up_sensor.get_value(), not self.down_sensor.get_value()

    def set_state(self, state):
        if state != self.state:
            self.previous_state = self.state
            self.completed = False
        self.state = state

    def update(self):
        up, down = self.read_sensors()
                
        if self.state == LifterStatus.MOVE_UP and up:
            self._handle_overrun(LifterStatus.MOVE_UP)
        elif self.state == LifterStatus.MOVE_DOWN and down:
            self._handle_overrun(LifterStatus.MOVE_DOWN)
        else:
            self.overrun_start_time = None
            self._apply_motion()

    def _handle_overrun(self, direction_val):
        if self.overrun_start_time is None:
            self.overrun_start_time = time.time()
        if (time.time() - self.overrun_start_time) < 1.0:
            self.dir_line.set_value(direction_val)
            self.enable()
        else:
            self.stop
            self.completed = True

    def _apply_motion(self):
        if self.state == LifterStatus.MOVE_UP:
            self.dir_line.set_value(LifterStatus.MOVE_UP)
            self.enable()
        elif self.state == LifterStatus.MOVE_DOWN:
            self.dir_line.set_value(LifterStatus.MOVE_DOWN)
            self.enable()
        else:
            self.stop()

    def enable(self):
        self.en_line.set_value(1)

    def stop(self):
        self.en_line.set_value(0)

    def cleanup(self):
        self.stop()
        self.dir_line.release()
        self.en_line.release()
        self.up_sensor.release()
        self.down_sensor.release()
        self.chip.close()
    
    def move(self, direction):
        self.dir_line.set_value(direction)
        self.en_line.set_value(1)
        
    def timed_move(self, direction, duration):
        try:
            duration = float(duration)
            if duration <= 0:
                raise ValueError("Duration must be positive.")
        except (TypeError, ValueError):
            print("Invalid duration value.")
            return

        self.move(direction)
        time.sleep(duration)
        self.stop()

    

