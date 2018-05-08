import time
from dronekit import connect, VehicleMode, LocationGlobalRelative, Command, LocationGlobal
from pymavlink import mavutil
import RPi.GPIO as GPIO
#import dronekit_sitl

GPIO.setmode(GPIO.BCM)
mover = int(0)

TRIG = 2
ECHO = 3

looptime = 0

GPIO.setup(TRIG,GPIO.OUT)
GPIO.setup(ECHO,GPIO.IN)

#sitl=dronekit_sitl.start_default(20.737641,-103.457018)
#connection_string = sitl.connection_string

def arm_and_takeoff(TargetAltitude):

    print ("Executing Takeoff")

    while not drone.is_armable:
        print ("Vehicle is not armable, waiting...")
        time.sleep(1)

    print ("Ready to arm")
    drone.mode = VehicleMode("GUIDED")
    drone.armed = True

    while not drone.armed:
        print("Waiting for arming...")
        time.sleep(1)

    print("Ready for takeoff, taking off...")
    drone.simple_takeoff(TargetAltitude)

    while True:
        Altitude = drone.location.global_relative_frame.alt
        print("Altitud", Altitude)
        time.sleep(1)

        if Altitude >= TargetAltitude * 0.95:
            print("ALtitude has been reached")
            break

#This function, as "arm_and_takeoff" is used to call some commands later, in this case they are used to...
#...change the drone position in 3 vectors, x, y and z.
def set_velocity_body(vehicle, vx, vy, vz):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
            0,
            0, 0,
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            0b0000111111000111, #-- BITMASK -> Consider only the velocities
            0, 0, 0,        #-- POSITION
            vx, vy, vz,     #-- VELOCITY
            0, 0, 0,        #-- ACCELERATIONS
            0, 0)
    vehicle.send_mavlink(msg)
    vehicle.flush()

def get_distance():
    GPIO.output(TRIG,False)
    print "Waiting for sensor"
    time.sleep(2)

    GPIO.output(TRIG, True)
    time.sleep(0.00001)
    GPIO.output(TRIG, False)

    while GPIO.input (ECHO)==0:
            pulse_start = time.time()

    while GPIO.input(ECHO)==1:
            pulse_end = time.time()

    pulse_duration = pulse_end - pulse_start
    dist = pulse_duration * 17150
    dist - round(dist, 2)

    GPIO.cleanup()

	GPIO.setmode(GPIO.BCM)

    TRIG = 2
    ECHO = 3
    GPIO.setup(TRIG,GPIO.OUT)
    GPIO.setup(ECHO,GPIO.IN)

    return dist
    """
    global looptime
    looptime = looptime + 1
    if looptime > 5:
        return 10

    return 100
    """
#This function is coordinated with "set_velocity_body", this is the one that reads the key that is being pressed.
#This part of the code is the one that puts the drone into movement with the keys.
#def key(event):
 #   if event.char == event.keysym: #-- standard keys
  #      if event.keysym == 'r':
   #         print ("Returning home")
    #        drone.mode = VehicleMode("RTL")
   # else:
       # if event.keysym == 'Up': set_velocity_body(drone, 5,0,0)
    #    elif event.keysym == 'Down':set_velocity_body(drone, -5,0,0)
     #   elif event.keysym == 'Left':set_velocity_body(drone, 0,-5,0)
      #  elif event.keysym == 'Right':set_velocity_body(drone, 0,5,0)

drone = connect('/dev/ttyAMA0', baud=57600, wait_ready=True)
#drone = connect('udpout:10.0.1.22:14551' , wait_ready=True)
#drone = connect('127.0.0.1:14551' , wait_ready=True)

# Take off to 10 m altitude
#arm_and_takeoff(2.10)

# Read the keyboard with tkinter: a little white board appears and you have to select the square to click the keys so they...
#... start reading the code and the movement.
#root = tk.Tk()
#print(">> Control the drone with the arrow keys. Press r for RTL mode")
#root.bind_all('<Key>', key)
#root.mainloop()

for i in range(10):

        distance = get_distance()
	print ("Distance:",distance,"cm")
	if distance <= 51:
	        print"La distancia es menor a medio metro"
	        print"El dron corre peligro de chocar"
	        mover = 0
	else:
	        print"La distancia es mayor a medio metro"
	        print"El dron no corre peligro"
	        mover = 1

	#if mover == 1: set_velocity_body(drone, 0,0,0)
	#elif mover  == 0: set_velocity_body(drone, -1,0,0)
	#elif event.keysym == 'Left':set_velocity_body(drone, 0,-1,0)
	#elif event.keysym == 'Right':set_velocity_body(drone, 0,1,0)
	time.sleep(3)
	mover = 1



print ("Returning home")
#drone.mode = VehicleMode("RTL")


#Retrieving the voltage of the battery from APM/Mission Planner and showing it to the user.
DroneBattery= drone.battery.voltage
print ('Drone Battery: %s V' % DroneBattery)

#Exiting the code.
drone.close()

#sitl.stop()
