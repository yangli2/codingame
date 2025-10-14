import sys
import math
import numpy as np

# Auto-generated code below aims at helping you parse
# the standard input according to the problem statement.

_ENGINE_CUTOFF_COEFF = 5.
_MIN_THRUST = 10
_MAX_THRUST = 100
_BOOST_DIST_THRESHOLD = 5000
_BOOST_ANGLE_THRESHOLD = 1

def normalize_angle(x: float):
    return x % (2. * math.pi)

def get_heading(x, y, ncx, ncy, nca):
    theta = normalize_angle(math.radians(nca))
    alpha = normalize_angle(math.atan2(ncy - y, ncx - x))
    return normalize_angle(math.pi / 2. + alpha - theta)

class Pod:
    def __init__(self):
        self.position = None
        self.velocity = np.array([0, 0])
        self.speed = 0
        self.next_checkpoint_id = 0
        self.checkpoints = []
        self.lap = -1
        self.lap_incremented = False

    def update(self, x: float, y: float, ncx: float, ncy: float, ncd: float, nca: float):
        # Calculate velocity
        new_position = np.array([x, y])
        if self.position is None:
            # Avoid computing a very large initial velocity
            self.position = new_position
        self.velocity = new_position - self.position
        self.speed = np.linalg.norm(self.velocity)
        self.position = new_position

        self.x = x
        self.y = y
        self.angle = nca
        self.heading = get_heading(x, y, ncx, ncy, nca)

        nc = (ncx, ncy)
        try:
            self.next_checkpoint_id = self.checkpoints.index(nc)
            if self.next_checkpoint_id == 0 and not self.lap_incremented:
                print(f"Incrementing lap to {self.lap}", file=sys.stderr)
                self.lap += 1
                self.lap_incremented = True
            elif self.next_checkpoint_id > 0:
                self.lap_incremented = False
        except ValueError:
            self.checkpoints.append(nc)
            self.next_checkpoint_id = len(self.checkpoints) - 1

    def calculate_optimal_action(self):
        """
        Calculates the best target and thrust for the pod to clear two checkpoints.

        Returns:
            A tuple of (target_vector, thrust_amount).
            - target_vector: A Vector2D representing the target point.
            - thrust_amount: A float for the thrust to apply (0.0 to 1.0).
        """
        nc = self.checkpoints[self.next_checkpoint_id]
        nnc = self.checkpoints[(self.next_checkpoint_id + 1) % len(self.checkpoints)]

        cp1_pos = np.array(nc)
        cp2_pos = np.array(nnc)

        # Vector from pod to checkpoint 1
        pod_to_cp1 = cp1_pos - self.position
        dist_to_cp1 = float(np.linalg.norm(pod_to_cp1))

        # Vector from checkpoint 1 to checkpoint 2
        cp1_to_cp2 = cp2_pos - cp1_pos
        dist_cp1_cp2 = float(np.linalg.norm(cp1_to_cp2))

        # Simple strategy: aim for a point away from cp1, extending from the line from cp2 to cp1.
        # The closer the pod is from cp1, the closer the target should be to cp1.
        print(f"next ckpt: {nc}, next next ckpt: {nnc}", file=sys.stderr)
        lookahead_distance = dist_to_cp1 * 0.30
        print(f"lookahead: {lookahead_distance}", file=sys.stderr)

        # The target point is away from cp1, the line segment from cp2 to cp1
        target_vector = cp1_pos - cp1_to_cp2 / dist_cp1_cp2 * lookahead_distance
        print(f"target: {target_vector}", file=sys.stderr)

        if math.fabs(self.angle) > 10:
            # Just aim at the ckpt if we are headed in the wrong direction
            target_vector = cp1_pos

        # --- Thrust Logic ---
        # The thrust is a scalar value (0.0 to 1.0)

        # 1. Base thrust: Apply full thrust if the pod is far from the target.
        # We use a tanh-like function to smoothly increase thrust with distance.
        distance_thrust = math.tanh(dist_to_cp1 / 500)
        print(f"distance_thrust: {distance_thrust}", file=sys.stderr)

        # # 2. Alignment thrust: Reduce thrust if the pod's heading is not aligned with the target.
        # # This helps with steering and prevents overshooting turns.
        # pod_to_target = target_vector - self.position
        # pod_to_target_dist = np.linalg.norm(pod_to_target)
        # if self.speed > 0 and pod_to_target_dist > 0:
        #     alignment_thrust_factor = np.dot(pod_to_target/pod_to_target_dist, self.velocity / self.speed)
        #     # We clamp the value to ensure it's not negative
        #     alignment_thrust_factor = max(float(_MIN_THRUST) / float(_MAX_THRUST), alignment_thrust_factor)
        # else:
        #     # Full thrust if not moving or no clear target
        #     alignment_thrust_factor = 1.0
        alignment_thrust_factor = 1.0
        print(f"alignment_thrust_factor: {alignment_thrust_factor}", file=sys.stderr)

        thrust_amount = distance_thrust * alignment_thrust_factor * float(_MAX_THRUST)
        print(f"thrust_amount: {thrust_amount}", file=sys.stderr)

        # Return the target vector and the final calculated thrust amount.
        return target_vector, thrust_amount

pod = Pod()
# game loop
while True:
    # next_checkpoint_x: x position of the next check point
    # next_checkpoint_y: y position of the next check point
    # next_checkpoint_dist: distance to the next checkpoint
    # next_checkpoint_angle: angle between your pod orientation and the direction of the next checkpoint
    pod_x, pod_y, ncx, ncy, ncd, nca = [int(i) for i in input().split()]
    opponent_x, opponent_y = [int(i) for i in input().split()]

    # Write an action using print
    # To debug: print("Debug messages...", file=sys.stderr)

    pod.update(pod_x, pod_y, ncx, ncy, ncd, nca)

    if pod.lap <= 0:
        if ncd < _ENGINE_CUTOFF_COEFF * pod.speed:
            print(f"ncd: {ncd}, speed: {pod.speed}", file=sys.stderr)
            thrust = _MIN_THRUST
        elif nca < -60 or nca > 60:
            thrust = 0
        elif ncd > _BOOST_DIST_THRESHOLD and nca > -_BOOST_ANGLE_THRESHOLD and nca < _BOOST_ANGLE_THRESHOLD:
            thrust = 'BOOST'
        else:
            thrust = _MAX_THRUST
        target_x, target_y = ncx, ncy
    else:
        target, thrust = pod.calculate_optimal_action()
        target_x, target_y = int(target[0]), int(target[1])
        thrust = int(thrust)

    # You have to output the target position
    # followed by the power (0 <= thrust <= 100)
    # i.e.: "x y thrust"
    print(f"{target_x} {target_y} {thrust}")
