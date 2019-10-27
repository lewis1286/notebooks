import numpy as np
from numpy.linalg import norm

class Boid():

    def __init__(self, position, field_length=1000):
        self.position = position
        self.velocity = (np.random.rand(2) - 0.5)*10
        self.acceleration = (np.random.rand(2) - 0.5)/2

        self.max_force = 0.3
        self.max_speed = 5
        self.perception = 100

        # playing field width / height
        self.field_length = field_length

    def update(self):
        self.position += self.velocity
        self.velocity += self.acceleration
        #limit
        if norm(self.velocity) > self.max_speed:
            self.velocity = self.velocity / norm(self.velocity) * self.max_speed

        self.acceleration = np.zeros(2)

    def apply_behavior(self, boids):
        alignment = self.align(boids)
        cohesion = self.cohesion(boids)
        separation = self.separation(boids)

        self.acceleration += alignment
        self.acceleration += cohesion
        self.acceleration += separation

    def edges(self):
        if self.position[0] > self.field_length:
            self.position[0] = 0
        elif self.position[0] < 0:
            self.position[0] = self.field_length

        if self.position[1] > self.field_length:
            self.position[1] = 0
        elif self.position[1] < 0:
            self.position[1] = self.field_length

    def align(self, boids):
        steering = np.zeros(2)
        total = 0
        avg_vector = np.zeros(2)
        for boid in boids:
            if norm(boid.position - self.position) < self.perception:
                avg_vector += boid.velocity
                total += 1
        if total > 0:
            avg_vector /= total
            avg_vector = (avg_vector / norm(avg_vector)) * self.max_speed
            steering = avg_vector - self.velocity

        return steering

    def cohesion(self, boids):
        steering = np.zeros(2)
        total = 0
        center_of_mass = np.zeros(2)
        for boid in boids:
            if norm(boid.position - self.position) < self.perception:
                center_of_mass += boid.position
                total += 1
        if total > 0:
            center_of_mass /= total
            vec_to_com = center_of_mass - self.position
            if norm(vec_to_com) > 0:
                vec_to_com = (vec_to_com / norm(vec_to_com)) * self.max_speed
            steering = vec_to_com - self.velocity
            if norm(steering) > self.max_force:
                steering = (steering /norm(steering)) * self.max_force

        return steering

    def separation(self, boids):
        steering = np.zeros(2)
        total = 0
        avg_vector = np.zeros(2)
        for boid in boids:
            distance = norm(boid.position - self.position)
            if 0 < distance < self.perception:
                diff = self.position - boid.position
                diff /= distance
                avg_vector += diff
                total += 1
        if total > 0:
            avg_vector /= total
            if norm(steering) > 0:
                avg_vector = (avg_vector / norm(steering)) * self.max_speed
            steering = avg_vector - self.velocity
            if norm(steering) > self.max_force:
                steering = (steering /norm(steering)) * self.max_force

        return steering
