#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import Point
from std_msgs.msg import Float64
from nav_msgs.msg import Path
from dynamic_reconfigure.server import Server
import dynamic_reconfigure

# Define the MOEPSO class
class MOEPSO:
    def __init__(self, num_particles, num_dimensions, max_iterations, search_space):
        self.num_particles = num_particles
        self.num_dimensions = num_dimensions
        self.max_iterations = max_iterations
        self.search_space = search_space
        self.particles = np.random.rand(num_particles, num_dimensions)
        self.velocities = np.zeros((num_particles, num_dimensions))
        self.best_position = np.zeros(num_dimensions)
        self.best_fitness = float('inf')
        self.current_iteration = 0

    def initialize_particles(self):
        for i in range(self.num_particles):
            for j in range(self.num_dimensions):
                self.particles[i, j] = np.random.uniform(self.search_space[j][0], self.search_space[j][1])
                self.velocities[i, j] = np.random.uniform(-0.1, 0.1)  # Example range for velocities

    def generate_particles(self):
        mutant_particles = np.zeros((self.num_particles, self.num_dimensions))
        for i in range(self.num_particles):
            mutant_particle = self.particles[i] + self.velocities[i]
            # Ensure mutant particle remains within search space
            mutant_particle = np.clip(mutant_particle, self.search_space[:, 0], self.search_space[:, 1])
            mutant_particles[i] = mutant_particle
        return mutant_particles

    def mutate_particles(self, mutant_particles):
        mutated_particles = np.zeros_like(mutant_particles)
        for i in range(self.num_particles):
            # Example mutation: Randomly perturb mutant particle
            mutation_factor = np.random.uniform(0.1, 0.2, size=self.num_dimensions)  # Example mutation factor
            mutated_particle = mutant_particles[i] + mutation_factor
            # Ensure mutated particle remains within search space
            mutated_particle = np.clip(mutated_particle, self.search_space[:, 0], self.search_space[:, 1])
            mutated_particles[i] = mutated_particle
        return mutated_particles

    def crossover_particles(self, particles, mutant_particles):
        trail_particles = np.zeros_like(particles)
        for i in range(self.num_particles):
            # Example crossover: Blend particles and mutant particles
            crossover_factor = np.random.uniform(0, 1, size=self.num_dimensions)  # Example crossover factor
            trail_particle = particles[i] * crossover_factor + mutant_particles[i] * (1 - crossover_factor)
            trail_particles[i] = trail_particle
        return trail_particles

    def fitness_function(self, particles):
        w1, w2, w3 = 0.3, 0.3, 0.4  # Weight factors for the three criteria
        shortest_path_fitness = self.shortest_path(particles)
        smooth_path_fitness = self.smooth_path(particles)
        safe_path_fitness = self.safe_path(particles)
        total_fitness = w1 * safe_path_fitness + w2 * smooth_path_fitness + w3 * shortest_path_fitness
        return total_fitness

    def shortest_path(self, particles):
        total_distance = 0
        for i in range(len(particles) - 1):
            dt = np.sqrt((particles[i + 1][0] - particles[i][0]) ** 2 + (particles[i + 1][1] - particles[i][1]) ** 2)
            total_distance += dt
        return total_distance

    def smooth_path(self, particles):
        # Calculate smooth path fitness
        total_difference = 0
        for i in range(len(particles) - 1):
            y_diff = particles[i + 1][1] - particles[i][1]
            x_diff = particles[i + 1][0] - particles[i][0]
            angle1 = np.arctan2(y_diff, x_diff)
            y_diff_dest = particles[-1][1] - particles[i][1]
            x_diff_dest = particles[-1][0] - particles[i][0]
            angle2 = np.arctan2(y_diff_dest, x_diff_dest)
            total_difference += np.abs(angle1 - angle2)
        return total_difference

    def safe_path(self, particles):
        # Calculate safe path fitness
        total_distance = 0
        for particle in particles:
            for obstacle_position in obstacle_positions:
                distance = np.linalg.norm(particle - obstacle_position)
                total_distance += distance
        return total_distance
        return np.sum(np.linalg.norm(particles - obstacle_positions, axis=1))

    def select_particles(self, particles, trail_particles):
        for i in range(self.num_particles):
            fitness_original = self.fitness_function(particles[i])
            fitness_trail = self.fitness_function(trail_particles[i])
            if fitness_trail < fitness_original:
                particles[i] = trail_particles[i]
                if fitness_trail < self.best_fitness:
                    self.best_position = trail_particles[i]
                    self.best_fitness = fitness_trail
        return particles

    def adjust_particle_values(self):
        for i in range(self.num_particles):
            for j in range(self.num_dimensions):
                lower_bound, upper_bound = self.search_space[j]
                if particles[i][j] < lower_bound:
                    particles[i][j] = lower_bound
                elif particles[i][j] > upper_bound:
                    particles[i][j] = upper_bound
        return particles

    def adjust_parameters(self):
        total = sum(parameters)
        if total != 1:
            parameters = [param / total for param in parameters]
        return parameters

    def optimize(self):
        while self.current_iteration < self.max_iterations:
            self.generate_particles()
            self.mutate_particles()
            self.crossover_particles()
            self.evaluate_fitness()
            self.select_particles()
            self.adjust_particle_values()
            self.adjust_parameters()
            self.current_iteration += 1

# Define the ROS node
class MOEPSONode:
    def __init__(self):
        rospy.init_node('moepso_node')
        self.server = Server(ConfigType, self.reconfigure_callback)
        self.moepso = MOEPSO(num_particles=50, num_dimensions=3, max_iterations=100)
        self.path_publisher = rospy.Publisher('/optimized_path', Path, queue_size=10)
        self.best_fitness_publisher = rospy.Publisher('/best_fitness', Float64, queue_size=10)
        self.run_moepso()

    def run_moepso(self):
        self.moepso.optimize()
        optimized_path = Path()
        optimized_path.header.frame_id = 'map'
        for particle in self.moepso.p
