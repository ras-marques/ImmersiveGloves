import board
import rp2pio
import adafruit_pioasm
import time

assembled = adafruit_pioasm.assemble("""
.program cs_sck_simulator
.side_set 1
.wrap_target:
    set x, 8 side 1
    pull side 1
    nop side 0
loop:
    out null 1 side 0
    set pins 1 side 0 [2]
    set pins 0 side 0
    jmp x-- loop side 0
.wrap
""")

sm = rp2pio.StateMachine(
    assembled,
    frequency=1000000,
    first_set_pin=board.GP0,
    first_sideset_pin=board.GP1,
    set_pin_count=2,
    sideset_pin_count=1
)

message = 'y'
test = bytes(message,'utf-8')

while True:
    print(test)
    sm.write(test)
    time.sleep(0.01)