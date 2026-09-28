#
# Sample Python client for the AI GP controller
#

import time

from setup import setup_components

# Modify these properties if you want to run the server remotely for example
SIM_SERVER_UDP_IP = "127.0.0.1"
SIM_SERVER_UDP_PORT = 14550

# time since sim started ms
system_boot_ms = int(time.time() * 1000)

# arbitrary shared data between the various components
shared_data: dict = {}


# setup components
components = setup_components(shared_data, system_boot_ms, SIM_SERVER_UDP_IP, SIM_SERVER_UDP_PORT)
controller = components['controller']
ts_loop = components['ts_loop']
mavlink_rx = components['mavlink_rx']
vision_rx = components['vision_rx']

print("Waiting for track data...", flush=True)
while 'gates' not in shared_data or len(shared_data['gates']) == 0:
    time.sleep(0.1)
print(f"Track data found: {len(shared_data['gates'])} gates", flush=True)

input("Press Enter to arm...")

print("Arming drone...", flush=True)
controller.arm()

# Fly to an arbitrary point in the local NED frame (x=North, y=East, z=Down; z negative = up)
# Test
controller.set_target_ned(5.0, 0.0, -3.0)

print("Starting control loop...", flush=True)
is_running = True
try:
    while is_running:
        controller.update()
finally:
    if controller.logger is not None:
        controller.logger.close()

    # exit
    ts_loop.get_thread_for_join().join(timeout=1.0)
    mavlink_rx.get_thread_for_join().join(timeout=1.0)
    vision_rx.get_thread_for_join().join(timeout=1.0)

    print("Client exited!", flush=True)
