
# Aidan Cotsakis
# Starded 2023/05/16

import pygame
import numpy as np
import math
import random
import os

maxBounces = 3

pygame.init()

windowSize = [512,512]

clock = pygame.time.Clock()
os.environ['SDL_VIDEO_CENTERED'] = '1'
win = pygame.display.set_mode(windowSize, pygame.NOFRAME)

quit = False

def normalize(vector):
	norm = np.linalg.norm(vector)
	if norm == 0:
	   return vector
	return vector / norm

def sphereCollision(rayPos, norm):
	shortestDistance = math.inf
	materialColour = np.array([0,0,0])
	materialEmmision = 0
	collisionCenter = None
	reflective = 0

	for sphere in spheres:
		pos = rayPos - sphere.pos

		detection = (pos @ norm)**2 - (pos @ pos - sphere.radius**2)

		if detection < 0:
			continue

		distance = -(pos @ norm) - math.sqrt(detection)

		if distance < shortestDistance and distance > 0:
			shortestDistance = distance
			materialColour = sphere.colour
			materialEmmision = sphere.emmision
			collisionCenter = sphere.pos
			reflective = sphere.reflective

	return shortestDistance, collisionCenter, materialColour, materialEmmision, reflective

def triangleCollision(rayPoint, rayUnitVector):
	shortestDistance = math.inf
	materialColour = np.array([0,0,0])
	materialEmmision = 0
	collisionNormal = None
	reflective = 0

	for triangle in triangles:
		rayToPoint = triangle.p1 - rayPoint # Calculate the vector from the ray's point to the first vertex of the triangle

		dot = np.dot(rayUnitVector, triangle.normal)

		if dot != 0:
			distance = np.dot(rayToPoint, triangle.normal) / dot # Calculate the distance from the ray's point to the triangle
		else:
			distance = math.inf

		intersectionPoint = rayPoint + distance * rayUnitVector # Calculate the intersection point

		# Compute barycentric coordinates
		v0 = triangle.p3 - triangle.p1
		v1 = triangle.p2 - triangle.p1
		v2 = intersectionPoint - triangle.p1

		dot00 = np.dot(v0, v0)
		dot01 = np.dot(v0, v1)
		dot02 = np.dot(v0, v2)
		dot11 = np.dot(v1, v1)
		dot12 = np.dot(v1, v2)

		invDenom = 1 / (dot00 * dot11 - dot01 * dot01)
		u = (dot11 * dot02 - dot01 * dot12) * invDenom
		v = (dot00 * dot12 - dot01 * dot02) * invDenom

		# Check if the point is inside the triangle
		if u >= 0 and v >= 0 and u + v <= 1:
			pass
		else:
			distance = math.inf

		if distance < shortestDistance and distance > 0:
			shortestDistance = distance
			materialColour = triangle.colour
			materialEmmision = triangle.emmision
			collisionNormal = triangle.normal
			reflective = triangle.reflective

	return shortestDistance, collisionNormal, materialColour, materialEmmision, reflective

def randomVector(normal):
	vec = normalize(np.array([np.random.normal(), np.random.normal(), np.random.normal()]))

	if normal @ vec < 0:
		return vec * -1
	else:
		return vec

def reflectionVector(vector, normal):
	vec = vector - 2*(vector @ normal)*normal

	return vec

def calculateRayPath(orgin, unit, bounces):
	rayColour = np.array([1.0,1.0,1.0])
	emmisionStrength = 0

	for i in range(bounces):

		sphere = True

		dist, collisionCenter, material, emmision, reflective = sphereCollision(orgin, unit)

		dist1, collisionNormal, material1, emmision1, reflective1 = triangleCollision(orgin, unit)

		if dist1 < dist:
			sphere = False
			dist = dist1
			material = material1
			emmision = emmision1
			reflective = reflective1

			if collisionNormal @ unit >= 0:
				collisionNormal *= -1
		
		if dist == math.inf:
			if emmisionStrength == 0:
				emmisionStrength = 0.1
			break

		rayColour *= material
		emmisionStrength = min(emmision,1)

		if emmisionStrength == 1:
			break

		orgin = orgin + unit*dist

		if sphere:
			if reflective != 0 and reflective > random.random():
				unit = reflectionVector(unit, normalize(orgin-collisionCenter))

			else:	
				unit = normalize(orgin-collisionCenter)
				unit = normalize(unit + randomVector(unit))
		else:
			if reflective != 0 and reflective > random.random():
				unit = reflectionVector(unit, collisionNormal)
			else:
				unit = normalize(collisionNormal + randomVector(collisionNormal))

	return rayColour * emmisionStrength * 255

class cameraObj(object):
	def __init__(self, pos, rot, fov, scale, pixels):
		self.pos = np.array(pos) #indicating where the focal point is (3x1 array)
		self.rot = rot #unused for now, here for rotating the sensor (3x1 array)
		self.fov = fov #a single number dictating how far the focus point is from the sensor (float)
		self.scale = scale #indicates how big the x axis of the unrotated sensor is (float)
		self.pixels = pixels #indicating the size of the pixels that will be distributed among the sensor (2x1 array)

		self.locateSensor()

	# calculates the topleft endpoint of the sensor as well as the axis lines
	def locateSensor(self):
		self.sensorY = np.array([0,1,0])*self.scale
		self.sensorZ = np.array([0,0,-1])*self.scale*self.pixels[1]/self.pixels[0]
		self.sensorCorner = self.pos+np.array([self.fov,0,0])-self.sensorY/2-self.sensorZ/2

	# input a pixel coordinate and get the normal vector associated with that pixel
	def getNormal(self, pixelPos):
		vector = self.sensorCorner + self.sensorY * pixelPos[0]/self.pixels[0] + self.sensorZ * pixelPos[1]/self.pixels[1] - self.pos

		return normalize(vector)

class sphereObj(object):
	def __init__(self, pos, radius, colour, emmision, reflective):
		self.pos = np.array(pos)
		self.radius = radius
		self.colour = np.array(colour)
		self.emmision = emmision
		self.reflective = reflective

class triangleObj(object):
	def __init__(self, p1, p2, p3, colour, emmision, reflective):
		self.p1 = np.array(p1)
		self.p2 = np.array(p2)
		self.p3 = np.array(p3)
		self.colour = np.array(colour)
		self.emmision = emmision
		self.reflective = reflective

		self.edge1 = self.p2 - self.p1
		self.edge2 = self.p3 - self.p1

		self.normal = normalize(np.cross(self.edge1, self.edge2))

def addPlane(point, v1, v2, colour, emmision, reflective, triangles):
	point = np.array(point)
	v1 = np.array(v1)
	v2 = np.array(v2)

	triangles.append(triangleObj(point, point + v1, point + v2, colour, emmision, reflective))
	triangles.append(triangleObj(point + v1 + v2, point + v1, point + v2, colour, emmision, reflective))

def addCube(point, v1, v2, v3, colour, emmision, reflective, triangles):
	point = np.array(point)
	v1 = np.array(v1)
	v2 = np.array(v2)
	v3 = np.array(v3)

	addPlane(point, v1, v2, colour, emmision, reflective, triangles)
	addPlane(point, v1, v3, colour, emmision, reflective, triangles)
	addPlane(point, v2, v3, colour, emmision, reflective, triangles)
	addPlane(point+v1, v2, v3, colour, emmision, reflective, triangles)
	addPlane(point+v2, v1, v3, colour, emmision, reflective, triangles)
	addPlane(point+v3, v1, v2, colour, emmision, reflective, triangles)

def detectQuit():
	for event in pygame.event.get():
			#exit
			if event.type == pygame.QUIT:
				pygame.quit()

# handles randering the frame, acts as a shader drawing a pixel at a time
def draw(passes):

	for i in range(windowSize[1]):
		for j in range(windowSize[0]):
			norm = cam.getNormal([j,i])
			colour = calculateRayPath(cam.pos, norm, maxBounces + 1)

			oldColour = np.array(win.get_at((j,i))[0:-1])

			newColour = (oldColour*passes + colour)/(passes+1)

			win.set_at((j,i), newColour)

		detectQuit()
	
		pygame.display.update()
		

	return passes + 1

cam = cameraObj([0.025,0,0.1], [0,0,0], 0.15, 0.3, windowSize)
spheres = [
sphereObj([0.3,0,0.1], 0.1, (1.0,1.0,1.0), 0, 0),
sphereObj([0.4,-0.2,0.1], 0.1, (1.0,0,0), 0, 0),
sphereObj([0.4,0.2,0.1], 0.1, (0,0,1.0), 0, 0),
sphereObj([-3,1,2], 3, (1.0,1.0,1.0), 1, 0),
sphereObj([0.5,0,-100], 100, (1.0,1.0,1.0), 0, 0.5),
]
triangles = []

passes = 0
# main loop
while True:

	detectQuit()

	passes = draw(passes)

	pygame.image.save(win, f"output/pass[{passes}].png")

# exit pygame
pygame.quit()
