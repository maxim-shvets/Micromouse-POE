"""Motor test -- runs both motors forward at increasing duty then stops."""
import board
import pwmio
import time

PIN_M1A = "D10"
PIN_M1B = "D9"
PIN_M2A = "D8"
PIN_M2B = "D7"

m1a = pwmio.PWMOut(getattr(board, PIN_M1A), frequency=1000)
m1b = pwmio.PWMOut(getattr(board, PIN_M1B), frequency=1000)
m2a = pwmio.PWMOut(getattr(board, PIN_M2A), frequency=1000)
m2b = pwmio.PWMOut(getattr(board, PIN_M2B), frequency=1000)

def set_motors(duty_l, duty_r):
    # duty in 0.0 - 1.0
    dl = int(min(1.0, max(0.0, duty_l)) * 65535)
    dr = int(min(1.0, max(0.0, duty_r)) * 65535)
    m1a.duty_cycle = dl
    m1b.duty_cycle = 0
    m2a.duty_cycle = 0
    m2b.duty_cycle = dr

def stop():
    m1a.duty_cycle = 0
    m1b.duty_cycle = 0
    m2a.duty_cycle = 0
    m2b.duty_cycle = 0

print("Motor test starting -- prop robot off ground!")
time.sleep(2)

for duty in (0.3, 0.5, 0.75, 1.0):
    print("Forward duty={:.0f}%".format(duty * 100))
    set_motors(duty, duty)
    time.sleep(1.5)
    stop()
    time.sleep(0.5)

print("Reverse")
m1a.duty_cycle = 0
m1b.duty_cycle = 65535
m2a.duty_cycle = 65535
m2b.duty_cycle = 0
time.sleep(1.5)
stop()

print("Done.")
