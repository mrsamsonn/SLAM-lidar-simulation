import env, sensors
import pygame

environment = env.buildEnvironment((600,1200))
environment.originalMap = environment.map.copy()
laser = sensors.LaserSensor(200, environment.originalMap, uncertainty=(0.5,0.01))
environment.map.fill((0,0,0)) #fill main map with black to hide everything
environment.infomap = environment.map.copy() #where pointCloud is drawn

running = True

while running :
    sensorON = False
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        if pygame.mouse.get_focused(): 
            #sensor is only on when mouse is inside window to prevent negative values
            sensorON = True
        elif not pygame.mouse.get_focused():
            sensorON = False
    
    if sensorON:
        position = pygame.mouse.get_pos()
        laser.position=position
        sensor_data = laser.sense_obstacles()
        environment.dataStorage(sensor_data)
        environment.show_sensorData()
    
    environment.map.blit(environment.infomap, (0,0))
    pygame.display.update()

