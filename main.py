import machine
import utime
from machine import I2C
import time
import ustruct


# Define I2C pins (SCL and SDA)
i2c = I2C(0, scl=machine.Pin(13), sda=machine.Pin(12), freq=100000)

tofLeftEnable = machine.Pin(9, machine.Pin.OUT)
tofRightEnable = machine.Pin(6, machine.Pin.OUT)
tofFrontEnable = machine.Pin(7, machine.Pin.OUT)
i2cBreakoutEnable = machine.Pin(8, machine.Pin.OUT)
tofLeftEnable.off()
tofRightEnable.off()
tofFrontEnable.off()
i2cBreakoutEnable.off()


class Sensor:
    def __init__(self, i2c, address=0x29):
        self.i2c = i2c
        self._address = address

        try:
            print(self.myRead16(0x0212))
            if self.myRead16(0x0212) == 0x29:
                self.address(self._address)
        except OSError:
            self.address(self._address)
            
        self.default_settings()
        self.init()

    def myWrite16(self, register, regValue):
        """ write a byte to specified 16 bit register """
        return self.i2c.writeto_mem(self._address, register, bytearray([regValue]), addrsize=16), 'big'

    def myRead16(self, register):
        """read 1 bit from 16 byte register"""
        # i2c.readfrom_mem(0x29, 0x0016, 1, addrsize=16)
        value = int.from_bytes(
            self.i2c.readfrom_mem(self._address, register, 1, addrsize=16),
            'big'
        )
        return value & 0xFFFF

    def init(self):
        if self.myRead16(0x0016) != 1:
            raise RuntimeError("Failure reset")

        # Recommended setup from the datasheet
     
        self.myWrite16(0x0207, 0x01)
        self.myWrite16(0x0208, 0x01)
        self.myWrite16(0x0096, 0x00)
        self.myWrite16(0x0097, 0xfd)
        self.myWrite16(0x00e3, 0x00)
        self.myWrite16(0x00e4, 0x04)
        self.myWrite16(0x00e5, 0x02)
        self.myWrite16(0x00e6, 0x01)
        self.myWrite16(0x00e7, 0x03)
        self.myWrite16(0x00f5, 0x02)
        self.myWrite16(0x00d9, 0x05)
        self.myWrite16(0x00db, 0xce)
        self.myWrite16(0x00dc, 0x03)
        self.myWrite16(0x00dd, 0xf8)
        self.myWrite16(0x009f, 0x00)
        self.myWrite16(0x00a3, 0x3c)
        self.myWrite16(0x00b7, 0x00)
        self.myWrite16(0x00bb, 0x3c)
        self.myWrite16(0x00b2, 0x09)
        self.myWrite16(0x00ca, 0x09)
        self.myWrite16(0x0198, 0x01)
        self.myWrite16(0x01b0, 0x17)
        self.myWrite16(0x01ad, 0x00)
        self.myWrite16(0x00ff, 0x05)
        self.myWrite16(0x0100, 0x05)
        self.myWrite16(0x0199, 0x05)
        self.myWrite16(0x01a6, 0x1b)
        self.myWrite16(0x01ac, 0x3e)
        self.myWrite16(0x01a7, 0x1f)
        self.myWrite16(0x0030, 0x00)

        #  writeReg System__Fresh_OUT_OF_Reset
        # self.myWrite16(0x0016, 0x00),

    def default_settings(self):
        # Enables polling for ‘New Sample ready’ when measurement completes
        self.myWrite16(0x0011, 0x10)
        self.myWrite16(0x010A, 0x30)  # Set Avg sample period
        self.myWrite16(0x003f, 0x46)  # Set the ALS gain - default is 0x46 ---- ADJUST THIS
        self.myWrite16(0x0031, 0xFF)  # Set auto calibration period
        # (Max = 255)/(OFF = 0)
        self.myWrite16(0x0040, 0x63)  # Set ALS integration time to 100ms ----- ADJUST THIS
        # perform a single temperature calibration
        self.myWrite16(0x002E, 0x01)

        # Optional settings from datasheet
        self.myWrite16(0x001B, 0x09)  # Set default ranging inter-measurement
        # period to 100ms
        self.myWrite16(0x003E, 0x0A)  # Set default ALS inter-measurement
        # period to 100ms
        self.myWrite16(0x0014, 0x24)  # Configures interrupt on ‘New Sample
        # Ready threshold event’

        # Additional settings defaults from community
        self.myWrite16(0x001C, 0x32)  # Max convergence time
        self.myWrite16(0x002D, 0x10 | 0x01)  # Range check enables
        self.myWrite16(0x0022, 0x7B)  # Eraly coinvergence estimate
        self.myWrite16(0x0120, 0x01)  # Firmware result scaler

    def identify(self):
        """Retrieve identification information of the sensor."""
        return {
            'model': self.myRead16(0x0000),
            'revision': (self.myRead16(0x0001), self.myRead16(0x0002)),
            'module_revision': (self.myRead16(0x0003),
                                self.myRead16(0x0004)),
            'date': self.myRead16(0x006),
            'time': self.myRead16(0x008),
        }

    def address(self, address=None):
        """Change the I2C address of the sensor."""
        if address is None:
            return self._address
        if not 8 <= address <= 127:
            raise ValueError("Wrong address")
        #self.myWrite16(0x0212, address)
        self.i2c.writeto_mem(0x29, 0x0212, bytearray([address]), addrsize=16)
        #self._set_reg8(0x0212, address)
        self._address = address

    def range(self):
        """Measure the distance in millimeters. Takes 0.01s."""
        self.myWrite16(0x0018, 0x01)  # Sysrange start
        time.sleep(0.01)
        return self.myRead16(0x0062)  # Result range valueimport ustruct
    
class MCP23008:
    IODIR = 0x00
    IPOL =  0x01
    GPPU =  0x06
    GPIO =  0x09

    def __init__(self, i2c, addr=None):
        self._addr = 0x20
        self.i2c = i2c
        self.ipol(0x000) # set normal polarity

    def _write(self, reg, value):
        self.i2c.writeto(self._addr, ustruct.pack('<BB', reg, value))

    def _read(self, reg):
        self.i2c.writeto(self._addr, ustruct.pack('<B', reg), stop=False)
        return self.i2c.readfrom(self._addr, 1)

    def output(self, val):
        self._write(self.GPIO, val)

    def input(self):
        return self._read(self.GPIO)

    def direction(self, val):
        self._write(self.IODIR, val)

    def gppu(self, val):
        self._write(self.GPPU, val)

    def ipol(self, val):
        self._write(self.IPOL, val)
    
    def display_on_7_segment(self, value):
        #MSB-LSB: dp, C, D, E, G, F, A, B
        seven_segment_values = [0b01110111, 0b01000001, 0b00111011, 0b01101011, 0b01001101, 0b01101110, 0b01111110, 0b01000011, 0b01111111, 0b01101111]
        result = 0x00

        # Invert the bits
        if type(value) is int:
            inverted_value = seven_segment_values[value] ^ 0xFF
            result = int(bin(inverted_value)[2:], 2)
        
        elif type(value) is list:
            if value[0] == 1:
                result |= 0b00000010
            if value[1] == 1:
                result |= 0b00000001
            if value[2] == 1:
                result |= 0b00001000
            if value[3] == 1:
                result |= 0b00000100
            inverted_value = result ^ 0xFF
            result = int(bin(inverted_value)[2:], 2)
        self.output(result)

class Tile:
    def __init__(self, position, explored, colour, north_wall, east_wall, south_wall, west_wall) -> None:
        self.position = position
        self.explored = explored
        self.colour = colour
        self.north_wall = north_wall
        self.east_wall = east_wall
        self.south_wall = south_wall
        self.west_wall = west_wall

    def possible_movements(self):
        directions = []
        if not self.north_wall:
            directions.append((0, 1))
        if not self.east_wall:
            directions.append((1, 0))
        if not self.south_wall:
            directions.append((0, -1))
        if not self.west_wall:
            directions.append((-1, 0))
        return directions
    

# initialise the i2c devices
tofLeftEnable.on()
tofLeft = Sensor(i2c, address=0x33)
tofRightEnable.on()
tofRight = Sensor(i2c, address=0x32)
tofFrontEnable.on()
tofFront = Sensor(i2c, address=0x31)

i2cBreakoutEnable.on()
mcp23008 = MCP23008(i2c)
mcp23008.direction(0x00)


# Define motor pins
motor_A_pwm_forward = machine.PWM(machine.Pin(19)) 
motor_A_pwm_backward = machine.PWM(machine.Pin(18)) 
motor_B_pwm_forward = machine.PWM(machine.Pin(20)) 
motor_B_pwm_backward = machine.PWM(machine.Pin(21))

# Set PWM frequency (Hz) for both motors
pwm_freq = 1000
motor_A_pwm_forward.freq(pwm_freq)
motor_A_pwm_backward.freq(pwm_freq)

motor_B_pwm_forward.freq(pwm_freq)
motor_B_pwm_backward.freq(pwm_freq)

# Define the counter
left_ticks = 0
right_ticks = 0

# Define the GPIO pins
left_tick_pin = machine.Pin(26, machine.Pin.IN, machine.Pin.PULL_DOWN)
right_tick_pin = machine.Pin(27, machine.Pin.IN, machine.Pin.PULL_DOWN)

# Interrupt handler functions for each pin
def left_tick_handler(pin):
    global left_ticks
    left_ticks += 1

def right_tick_handler(pin):
    global right_ticks
    right_ticks += 1

# Configure pin change interrupts on rising edge for each pin
left_tick_pin.irq(trigger=machine.Pin.IRQ_RISING, handler=left_tick_handler)
right_tick_pin.irq(trigger=machine.Pin.IRQ_RISING, handler=right_tick_handler)

def stop():
    motor_A_pwm_forward.duty_u16(0)
    motor_A_pwm_backward.duty_u16(0)
    motor_B_pwm_forward.duty_u16(0)
    motor_B_pwm_backward.duty_u16(0)

def reset_ticks():
    global left_ticks
    global right_ticks
    left_ticks, right_ticks = 0,0

def move_distance(distance, speed):
    '''
    specify a distance to move forwards or backwards in a straight line

    distance: how far to move in mm (positive or negative)
    speed: 0-100

    This is a proportional controller to try to get the wheels to turn the same amount.
    This means if one has more force or is less powerful, the other will slow down or speed up to keep it straight
    This works unreasonably well
    '''
    if distance > 0:
        while left_ticks < distance or right_ticks < distance:
            error = left_ticks - right_ticks
            #print(error)
            Kp = -5
            motor_A_pwm_forward.duty_u16(int((speed + error * Kp) * 65535 / 100))
            motor_B_pwm_forward.duty_u16(int((speed - error * Kp) * 65535 / 100))

            if left_ticks >= distance:
                motor_A_pwm_forward.duty_u16(0)
                motor_A_pwm_backward.duty_u16(0)
            if right_ticks >= distance:
                motor_B_pwm_forward.duty_u16(0)
                motor_B_pwm_backward.duty_u16(0)
    reset_ticks()


def turn_degrees(angle, speed):
    '''
    turns to angle (-180, 180) at speed (0,100)

    angle: 0 is forwards, +ve is clockwise, -ve is anticlockwise
    '''
    required_ticks = int(0.78 * abs(angle))

    trim = 1.1

    if angle == -90:
        print("turn left")
    elif angle == 90:
        print("turn_right")
    elif angle == 180 or angle == -180:
        print("turn 180")
    
    while left_ticks < required_ticks*trim or right_ticks < required_ticks*trim:
        error = left_ticks - right_ticks
        #print(error)
        Kp = -5

        if angle > 0:
            motor_A_pwm_forward.duty_u16(int((speed + error * Kp) * 65535 / 100))
            motor_B_pwm_backward.duty_u16(int((speed - error * Kp) * 65535 / 100))
        else:
            motor_B_pwm_forward.duty_u16(int((speed + error * Kp) * 65535 / 100))
            motor_A_pwm_backward.duty_u16(int((speed - error * Kp) * 65535 / 100))

        if left_ticks >= required_ticks:
            motor_A_pwm_forward.duty_u16(0)
            motor_A_pwm_backward.duty_u16(0)
        if right_ticks >= required_ticks:
            motor_B_pwm_forward.duty_u16(0)
            motor_B_pwm_backward.duty_u16(0)
    stop()
    reset_ticks()


def move_forward_tiles(speed, tiles_to_move):
    '''
    while encoders < tile * something or not something infront
        case 1 - both tof < 100
            keep them equal
        case 2 - left tof < 100
            keep left at value
        case 3 - right tof < 100
            keep right at value
        case 4 - no tof data
            drive straight
    '''

    Kp = -2
    error = 0
    distance_to_wall = 80
    min_tof_threshold = 100

    trim = 0.95

    while (left_ticks < 0.94*180*tiles_to_move*trim or right_ticks < 0.94*180*tiles_to_move*trim) and tofFront.range() > 20:
        #print(left_ticks)
        #print(0.94*180*tiles_to_move)
        left_dist = tofLeft.range()
        right_dist = tofRight.range()
        front_dist = tofFront.range()
    
        # case 1
        if left_dist < min_tof_threshold and right_dist < min_tof_threshold:
            #keep equal
            error = left_dist - right_dist
        # case 2
        elif left_dist < min_tof_threshold and right_dist > min_tof_threshold:
            # hold against left
            error = left_dist - distance_to_wall
        # case 3
        elif right_dist < min_tof_threshold and left_dist > min_tof_threshold:
            # hold against right
            error = distance_to_wall - right_dist
        # case 4 
        else:
            # drive straigt
            error = left_ticks - right_ticks
            
        motor_A_pwm_forward.duty_u16(int((speed + (error * Kp)) * 65535 / 100))
        motor_B_pwm_forward.duty_u16(int((speed - (error * Kp)) * 65535 / 100))


        walls = [0,0,0,0]
        if left_dist < 100:
            walls[3] = 1
        if front_dist < 100:
            walls[0] = 1
        if right_dist < 100:
            walls[1] = 1
        mcp23008.display_on_7_segment(walls)

    stop()
    reset_ticks()
    return


def dijkstra(maze, start, end):
    rows, cols = len(maze), len(maze[0])
    starting_location = start
    end_location = end

    # Initialize predecessors and distances dictionaries
    predecessors = {starting_location: None}
    distances = {starting_location: 0}

    queue = [starting_location]

    while queue:
        current_position = queue.pop(0)

        for direction in maze[len(maze)-1-current_position[1]][current_position[0]].possible_movements():
            new_position = (current_position[0] + direction[0], current_position[1] + direction[1])

            # Check if the new position is within the maze bounds
            if 0 <= new_position[0] < rows and 0 <= new_position[1] < cols:
                new_distance = distances[current_position] + 1

                if new_distance < distances.get(new_position, float('inf')):
                    distances[new_position] = new_distance
                    predecessors[new_position] = current_position
                    queue.append(new_position)

    shortest_distance = distances[end_location]
    # Reconstruct path
    end = end_location
    path = [end]

    while predecessors[end] is not None:
        end = predecessors[end]
        path.insert(0, end)

    return shortest_distance, path

def global_transform(direction):
    global_direction = (0,0)
    if facing == 0:
        global_direction = direction
    elif facing == 1:
        if direction == (0,1):
            global_direction = (1,0)
        elif direction == (1,0):
            global_direction = (0,-1)
        elif direction == (-1,0):
            global_direction = (0,1)
        else:
            global_direction = (-1,0)
    elif facing == 2:
        if direction == (0,1):
            global_direction = (0,-1)
        elif direction == (1,0):
            global_direction = (-1,0)
        elif direction == (-1,0):
            global_direction = (1,0)
        else:
            global_direction = (0,1)
    else:
        if direction == (0,1):
            global_direction = (-1,0)
        elif direction == (1,0):
            global_direction = (0,1)
        elif direction == (-1,0):
            global_direction = (0,-1)
        else:
            global_direction = (1,0)
    return global_direction

def print_maze(maze):
    for i in range(len(maze)):
        row = []
        for j in range(len(maze[i])):
            row.append((bool(maze[i][j].explored), int(maze[i][j].north_wall), int(maze[i][j].east_wall), int(maze[i][j].south_wall), int(maze[i][j].west_wall)))
        print(row)

try:
    maze = []
    for i in range(9):
        maze.append([])
        for j in range(9):
            maze[i].append(Tile((j,8-i),0,0,0,0,0,0))


    starting_pos = [0,0]
    current_pos = starting_pos
    facing = 0 # 0 is North, 1 is East, -1 is West, 2 is South
    while True:
        # break if all explored
        if 1 == 0:
            break
        # send error if out of bounds

        left_dist = tofLeft.range()
        right_dist = tofRight.range()
        front_dist = tofFront.range()
        print(left_dist, right_dist, front_dist)

        #        N E S W
        walls = [0,0,0,0]
        if left_dist < 100:
            walls[3] = 1
        if front_dist < 100:
            walls[0] = 1
        if right_dist < 100:
            walls[1] = 1
        mcp23008.display_on_7_segment(walls)        
        
        # correction for changing directions
        walls =  walls[facing:] + walls[:facing]

        # update maze map
        if current_pos == [0,0]:
            maze[8][0] = Tile((0,0), 1, 0, walls[0], walls[1], 1, 1)
        else:
            maze[8-current_pos[1]][current_pos[0]] = Tile(current_pos, 1, 0, walls[0], walls[1], walls[2], walls[3])
        
        print_maze(maze)

        # add possible directions to list
        possible_directions = []
        if left_dist == 255:
            possible_directions.append((-1, 0))
        if front_dist == 255:
            possible_directions.append((0, 1))
        if right_dist == 255:
            possible_directions.append((1, 0))
        possible_directions.append((0,-1))
        
        #possible_directions =  possible_directions[facing:] + possible_directions[:facing]
        print("possible directions:  " + str(possible_directions))
        
        direction_to_move = None
        # eliminate possible movements that would go into an already explored tile if len > 1
        if len(possible_directions) > 1:
            for direction in possible_directions:
                # convert direction into global direction based on facing
                global_direction = global_transform(direction)
                # check if it's been explored

                if maze[8-(current_pos[1]+global_direction[1])][current_pos[0]+global_direction[0]].explored == False:
                    direction_to_move = direction
                    print('direction relative to robot: ' + str(direction))

                    # update position
                    current_pos[0] += global_direction[0]
                    current_pos[1] += global_direction[1]
                    break

        # otherwise just pick 1st in list
        if direction_to_move is None:
            direction_to_move = possible_directions[0]
            global_direction = global_transform(direction_to_move)
            current_pos[0] += global_direction[0]
            current_pos[1] += global_direction[1]

        print("current pos:  " + str(current_pos))
        print("facing:  " + str(facing))
        
        # move to next tile
        if direction_to_move == (0,1):
            pass
        elif direction_to_move == (-1, 0):
            turn_degrees(-90,80)
            facing -= 1
            if facing < -1:
                facing = 2
        elif direction_to_move == (1,0):
            turn_degrees(90,80)
            facing += 1
            if facing > 2:
                facing = -1
        else:
            turn_degrees(90,80)
            time.sleep(0.5)
            turn_degrees(90,80)
            if facing < 1:
                facing += 2
            else:
                facing -= 2
        
        move_forward_tiles(80, 1)
        time.sleep(1)




    shortest_path_lengths, shortest_path = dijkstra(maze, (0,0), (1,2))
    print("Shortest Path Lengths:", shortest_path_lengths)
    print("Shortest Path:", shortest_path)


    move_forward_tiles(50, 6)

    
    
finally:
    stop()
