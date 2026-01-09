import serial.tools.list_ports
import serial
import time

def find_ports():
    ports = serial.tools.list_ports.comports()
    for port in ports:
        print(f"{port.device}: {port.description}")

ARDUINO_PORT = "/dev/cu.usbmodem3101"
ENCODING = 'utf-8'
ser = serial.Serial(ARDUINO_PORT, 9600, timeout=1)
print(f"Connected to {ser.name}")

def read_all_output():
    global ser
    line = ser.readline()
    while line:
        print(line.decode(ENCODING).strip())
        line = ser.readline()

def send_command(message: str):
    global ser
    ser.write(message.encode(ENCODING))
    time.sleep(1)
    read_all_output()

def move_to_coordinate(x: int, y: int):
    command_str = "z {x_param} {y_param}\n".format(x_param = x, y_param = y)
    print(command_str)
    send_command(command_str)

def get_next_pos(curr_pos: int, target_pos: int, step: int):
    if step > 0:
        return min(curr_pos + step, target_pos)
    else:
        return max(curr_pos + step, target_pos)

def get_step(from_pos: int, to_pos: int):
    STEP_MAGNITUDE = 25
    if from_pos == to_pos:
        return 0
    return STEP_MAGNITUDE if from_pos < to_pos else -STEP_MAGNITUDE

def translate(from_x: int, from_y: int, to_x: int, to_y: int):
    global curr_x, curr_y

    step_x = get_step(from_x, to_x) 
    step_y = get_step(from_y, to_y)
    # print("step_x " + str(step_x))
    # print("step_y " + str(step_y))

    while curr_x != to_x or curr_y != to_y:
        next_x = get_next_pos(curr_x, to_x, step_x)
        next_y = get_next_pos(curr_y, to_y, step_y)
        move_to_coordinate(next_x, next_y)
        curr_x = next_x
        curr_y = next_y

def translate_to(to_x: int, to_y: int):
    global curr_x, curr_y
    translate(curr_x, curr_y, to_x, to_y)

def draw_rectangle(width: int, height: int):
    global curr_x, curr_y
    init_x = curr_x
    final_x = curr_x + width
    init_y = curr_y
    final_y = curr_y - height 
    translate_to(final_x, curr_y)
    translate_to(final_x, final_y)
    translate_to(init_x, final_y)
    translate_to(init_x, init_y)


read_all_output()

curr_x = 0
curr_y = 700

# draw_rectangle(100, 50)
draw_rectangle(200, 200)

# translate_to(-100, 350)
# translate_to(0, 400)
# translate_to(100, 350)
# translate_to(0, 300)

