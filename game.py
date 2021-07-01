#
# game.py
#
# Simple tester for the Roomba Interface
# from create.py
#
# Adjust your ROOMBA_PORT if necessary.
# python game.py 
# starts a pygame window from which the 
# Roomba can be controlled with w/a/s/d.
# Use this file to play with the sensors.
import os, sys
import pygame
import create
import time

# Change the roomba port to whatever is on your
# machine. On a Mac it's something like this.
# On Linux it's usually tty.USB0 and on Win
# its to serial port.
ROOMBA_PORT = "/dev/tty.usbserial-DA017V6X"



robot = create.Create(ROOMBA_PORT, BAUD_RATE=115200)
robot.toSafeMode()
#robot.printSensors()

pygame.init()
size = width, height = 800, 600
screen = pygame.display.set_mode(size)
pygame.display.set_caption('Roomba Test')

img_roomba_top = pygame.image.load(os.path.join('img', 'roomba.png'))

# Fill background
background = pygame.Surface(screen.get_size())
background = background.convert()
background.fill((250, 250, 250))

# Display some text
#font = pygame.font.Font(None, 18)
font = pygame.font.SysFont("calibri",16)

# Blit everything to the screen
# screen.blit(background, (0, 0))
# pygame.display.flip()


MAX_FORWARD = 50 # in cm per second
MAX_ROTATION = 200 # in cm per second
SPEED_INC = 10 # increment in percent
# start 50% speed


lb_left = robot.senseFunc(create.LIGHTBUMP_LEFT)
lb_front_left = robot.senseFunc(create.LIGHTBUMP_FRONT_LEFT)
lb_center_left = robot.senseFunc(create.LIGHTBUMP_CENTER_LEFT)
lb_center_right = robot.senseFunc(create.LIGHTBUMP_CENTER_RIGHT)
lb_front_right = robot.senseFunc(create.LIGHTBUMP_FRONT_RIGHT)
lb_right = robot.senseFunc(create.LIGHTBUMP_RIGHT)
#dist_fun = robot.senseFunc(create.DISTANCE)


def main():
	FWD_SPEED = MAX_FORWARD/2
	ROT_SPEED = MAX_ROTATION/2

	robot_dir = 0
	robot_rot = 0
	side_brush = 0
	main_brush = 0
	vacuum = 0

	robot.resetPose()
	px, py, th = robot.getPose()

	while True:
		senses = robot.sensors([create.WALL_SIGNAL, create.WALL_IR_SENSOR, create.LEFT_BUMP, create.RIGHT_BUMP, create.ENCODER_LEFT, create.ENCODER_RIGHT, create.CLIFF_LEFT_SIGNAL, create.CLIFF_FRONT_LEFT_SIGNAL, create.CLIFF_FRONT_RIGHT_SIGNAL, create.CLIFF_RIGHT_SIGNAL, create.DIRT_DETECTED])
		update_roomba = False
		for event in pygame.event.get():
			if event.type == pygame.QUIT:
				return
			if event.type == pygame.KEYDOWN:
				if event.key == pygame.K_w:
					robot_dir+=1
					update_roomba = True
				if event.key == pygame.K_s:
					robot_dir-=1
					update_roomba = True
				if event.key == pygame.K_a:
					robot_rot+=1
					update_roomba = True
				if event.key == pygame.K_d:
					robot_rot-=1
					update_roomba = True
				if event.key == pygame.K_ESCAPE:
					pygame.quit()
					return
				if event.key == pygame.K_SPACE:
					robot.resetPose()
					px, py, th = robot.getPose()
				if event.key == pygame.K_UP:
					update_roomba = True
					FWD_SPEED += MAX_FORWARD*SPEED_INC/100
					if FWD_SPEED>MAX_FORWARD:
						FWD_SPEED = MAX_FORWARD
					ROT_SPEED += MAX_ROTATION*SPEED_INC/100
					if ROT_SPEED > MAX_ROTATION:
						ROT_SPEED = MAX_ROTATION
				if event.key == pygame.K_DOWN:
					update_roomba = True
					FWD_SPEED -= MAX_FORWARD*SPEED_INC/100
					if FWD_SPEED<0:
						FWD_SPEED = 0
					ROT_SPEED -= MAX_ROTATION*SPEED_INC/100
					if ROT_SPEED<0:
						ROT_SPEED = 0
				if event.key == pygame.K_m:
					update_roomba = True
					if (pygame.key.get_mods() & pygame.KMOD_SHIFT) :
						main_brush = 1
					else:
						main_brush = -1
					
				if event.key == pygame.K_v:
					vacuum = 1
					update_roomba = True
					
				if event.key == pygame.K_o:
					if (pygame.key.get_mods() & pygame.KMOD_SHIFT):
						side_brush = 1
					else:
						side_brush = -1
					update_roomba = True
			if event.type == pygame.KEYUP:
				if event.key == pygame.K_w or event.key == pygame.K_s:
					robot_dir=0
					update_roomba = True
				if event.key == pygame.K_a or event.key == pygame.K_d:
					robot_rot=0
					update_roomba = True
				if event.key == pygame.K_m:
					main_brush = 0
					update_roomba = True
				if event.key == pygame.K_o:
					side_brush = 0
					update_roomba = True
				if event.key == pygame.K_v:
					vacuum = 0
					update_roomba = True

		if update_roomba == True:
			#robot.sensors([create.POSE])		
			robot.go(robot_dir*FWD_SPEED,robot_rot*ROT_SPEED)
			robot.motors(side_brush, main_brush, vacuum)
			time.sleep(0.1)

		# done with the actual roomba stuff
		# now print.
		screen.blit(background, (0, 0))
		screen.blit(img_roomba_top, (0,0))
		#Light Bump
		screen.blit(font.render("{}".format(lb_left()), 1, (10, 10, 10)), (112, 136))
		screen.blit(font.render("{}".format(lb_front_left()), 1, (10, 10, 10)), (159, 62))
		screen.blit(font.render("{}".format(lb_center_left()), 1, (10, 10, 10)), (228, 19))

		screen.blit(font.render("{}".format(lb_center_right()), 1, (10, 10, 10)), (457, 19))
		screen.blit(font.render("{}".format(lb_front_right()), 1, (10, 10, 10)), (484, 54))
		screen.blit(font.render("{}".format(lb_right()), 1, (10, 10, 10)), (469, 115))
		#Wall Sensors
		screen.blit(font.render("{}".format(senses[create.WALL_IR_SENSOR]), 1, (10, 10, 10)), (376, 396))
		screen.blit(font.render("{}".format(senses[create.WALL_SIGNAL]), 1, (10, 10, 10)), (376, 416))
		#Bumpers
		screen.blit(font.render("{}".format(senses[create.LEFT_BUMP]), 1, (10, 10, 10)), (142, 396))
		screen.blit(font.render("{}".format(senses[create.RIGHT_BUMP]), 1, (10, 10, 10)), (142, 416))
		#Encoders
		screen.blit(font.render("{}".format(senses[create.ENCODER_LEFT]), 1, (10, 10, 10)), (635, 396))
		screen.blit(font.render("{}".format(senses[create.ENCODER_RIGHT]), 1, (10, 10, 10)), (635, 416))
		#Cliff Sensors
		screen.blit(font.render("{}".format(senses[create.CLIFF_LEFT_SIGNAL]), 1, (10, 10, 10)), (635, 16))
		screen.blit(font.render("{}".format(senses[create.CLIFF_FRONT_LEFT_SIGNAL]), 1, (10, 10, 10)), (635, 35))
		screen.blit(font.render("{}".format(senses[create.CLIFF_FRONT_RIGHT_SIGNAL]), 1, (10, 10, 10)), (635, 54))
		screen.blit(font.render("{}".format(senses[create.CLIFF_RIGHT_SIGNAL]), 1, (10, 10, 10)), (635, 73))

		screen.blit(font.render(" Fwd speed: {:04.2f} cm/sec (change with Up/Down)".format(FWD_SPEED), 1, (10, 10, 10)), (50, 450))
		screen.blit(font.render(" Rot speed: {:04.2f} cm/sec".format(ROT_SPEED), 1, (10, 10, 10)), (50, 470))
		
		px, py, th = robot.getPose()
		screen.blit(font.render("Estimated X-Position: {:04.2f} (cm from start)".format(px), 1, (10, 10, 10)), (450, 450))
		screen.blit(font.render("Estimated Y-Position: {:04.2f} (cm from start)".format(py), 1, (10, 10, 10)), (450, 470))
		screen.blit(font.render("  Estimated Rotation: {:03.2f} (in degree)".format(th), 1, (10, 10, 10)), (450, 490))
		screen.blit(font.render("       Dirt Detected: {}".format(senses[create.DIRT_DETECTED]), 1, (10, 10, 10)), (450, 510))	

		screen.blit(font.render("Move Roomba with w/a/s/d, adjust speed with UP/DOWN, reset pos with SPACE, and ESC to quit.", 1, (10, 10, 10)), (10, 560))
		screen.blit(font.render("o activates the sidebrush, m the main, v the vacuum. Holding shift could be used for reversing the the direction of the brushes.", 1, (10, 10, 10)), (10, 575))

		pygame.display.flip()
		
if __name__ == '__main__': 
	try:
		main()
	except Error as err:
		print (err)
	robot.go(0,0)
	robot.close()
