#!/usr/bin/env python3
import math
import random

class Particle:
    def __init__(self, x, y, theta):
        """
        Initialize the particle with an initial pose (x, y, theta).
        The weight is initialized to 1.0 as a starting assumption.
        """
        self.x = x        # Particle's x position
        self.y = y        # Particle's y position
        self.theta = theta  # Particle's orientation (theta)
        self.weight = 1.0  # Initial weight (uniform)

    def set_weight(self, weight):
        """
        Set the weight of the particle based on the likelihood from the measurement update.
        """
        self.weight = weight

    def get_pose(self):
        """
        Returns the current pose of the particle as a tuple (x, y, theta).
        """
        return (self.x, self.y, self.theta)

    def update_pose(self, delta_x, delta_y, delta_theta):
        """
        Update the particle's pose using a motion model (movement from odometry or control input).
        The deltas are typically based on `cmd_vel` or odometry updates.
        The motion is noisy, so we add a small Gaussian noise to simulate uncertainty.
        """
        noise_std = 0.1  # Standard deviation for noise

        self.x += delta_x + random.gauss(0, noise_std)
        self.y += delta_y + random.gauss(0, noise_std)
        self.theta += delta_theta + random.gauss(0, noise_std)

        # Normalize the angle to keep it within [-pi, pi]
        self.theta = (self.theta + math.pi) % (2 * math.pi) - math.pi

    def compute_distance(self, other_particle):
        """
        Compute the Euclidean distance between two particles (for debugging or diagnostics).
        """
        dx = self.x - other_particle.x
        dy = self.y - other_particle.y
        return math.sqrt(dx ** 2 + dy ** 2)

    def get_weight(self):
        """
        Returns the current weight of the particle.
        """
        return self.weight
