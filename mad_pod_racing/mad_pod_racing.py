import math
import logging

logging.basicConfig(level=logging.DEBUG, format='%(asctime)s - %(levelname)s - %(message)s')

POD_RADIUS = 400
CHECKPOINT_RADIUS = 600

class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def distance2(self, other):
        return (self.x - other.x)**2 + (self.y - other.y)**2

    def distance(self, other):
        return math.sqrt(self.distance2(other))

class Unit(Point):
    def __init__(self, x, y, id, r, vx, vy):
        super().__init__(x, y)
        self.id = id
        self.r = r
        self.vx = vx
        self.vy = vy

    def collision(self, other):
        dist = self.distance(other)
        if dist <= self.r + other.r:
            return Collision(self, other, 0.0)
        
        if self.vx == other.vx and self.vy == other.vy:
            return None

        x1, y1, r1 = self.x, self.y, self.r
        x2, y2, r2 = other.x, other.y, other.r
        vx1, vy1 = self.vx, self.vy
        vx2, vy2 = other.vx, other.vy

        a = vx1**2 - 2*vx1*vx2 + vx2**2 + vy1**2 - 2*vy1*vy2 + vy2**2
        b = 2 * (x1*vx1 - x1*vx2 - x2*vx1 + x2*vx2 + y1*vy1 - y1*vy2 - y2*vy1 + y2*vy2)
        c = x1**2 - 2*x1*x2 + x2**2 + y1**2 - 2*y1*y2 + y2**2 - (r1+r2)**2

        delta = b**2 - 4*a*c

        if delta < 0:
            return None

        t = (-b - math.sqrt(delta)) / (2*a)

        if t <= 0 or t > 1.0:
            return None

        return Collision(self, other, t)


class Checkpoint(Unit):
    def __init__(self, x, y, id, r):
        super().__init__(x, y, id, r, 0, 0)

class Pod(Unit):
    def __init__(self, x, y, id, r, vx, vy, angle, next_check_point_id):
        super().__init__(x, y, id, r, vx, vy)
        self.angle = angle
        self.next_check_point_id = next_check_point_id
        self.timeout = 100
        self.shield = 0
        self.checked = 0

    def get_angle(self, p):
        d = self.distance(p)
        if d == 0:
            return 0.0
        dx = (p.x - self.x) / d
        dy = (p.y - self.y) / d
        a = math.acos(dx)
        if dy < 0:
            a = 2 * math.pi - a
        return a

    def diff_angle(self, p):
        a = self.get_angle(p)
        da = a - self.angle
        if da > math.pi:
            da -= 2 * math.pi
        if da < -math.pi:
            da += 2 * math.pi
        return da

    def rotate(self, p):
        da = self.diff_angle(p)
        if da > math.radians(18):
            da = math.radians(18)
        elif da < -math.radians(18):
            da = -math.radians(18)
        self.angle += da

    def boost(self, thrust):
        if self.shield > 0:
            return
        
        ra = self.angle
        self.vx += math.cos(ra) * thrust
        self.vy += math.sin(ra) * thrust

    def move(self, t):
        self.x += self.vx * t
        self.y += self.vy * t

    def end(self):
        self.x = round(self.x)
        self.y = round(self.y)
        self.vx = int(self.vx * 0.85)
        self.vy = int(self.vy * 0.85)
        self.timeout -= 1
        if self.shield > 0:
            self.shield -= 1
    
    def play(self, p, thrust):
        self.rotate(p)
        self.boost(thrust)
        self.move(1.0)
        self.end()

    def bounce(self, other):
        if isinstance(other, Checkpoint):
            self.checked += 1
            self.timeout = 100
            self.next_check_point_id = (self.next_check_point_id + 1) % 3 # Assumes 3 checkpoints
            return

        m1 = 10 if self.shield > 0 else 1
        m2 = 10 if isinstance(other, Pod) and other.shield > 0 else 1
        m_total = m1 + m2
        
        nx = self.x - other.x
        ny = self.y - other.y
        
        d = math.sqrt(nx**2 + ny**2)
        
        if d == 0:
            unx = 1
            uny = 0
        else:
            unx = nx / d
            uny = ny / d

        # Resolve overlap
        overlap = self.r + other.r - d
        if overlap > 0:
            separation_dist = (overlap + 0.001) / 2
            self.x += unx * separation_dist
            self.y += uny * separation_dist
            other.x -= unx * separation_dist
            other.y -= uny * separation_dist
        
        p = 2 * (self.vx * unx + self.vy * uny - other.vx * unx - other.vy * uny) / m_total
        
        self.vx -= p * m2 * unx
        self.vy -= p * m2 * uny
        other.vx += p * m1 * unx
        other.vy += p * m1 * uny

        impulse = math.sqrt((self.vx - other.vx)**2 + (self.vy - other.vy)**2)
        if impulse < 120 and impulse > 0:
            impulse_factor = 120 / impulse
            self.vx *= impulse_factor
            self.vy *= impulse_factor
            other.vx *= impulse_factor
            other.vy *= impulse_factor


class Collision:
    def __init__(self, a, b, t):
        self.a = a
        self.b = b
        self.t = t

class Game:
    def __init__(self, checkpoints, pods):
        self.checkpoints = checkpoints
        self.pods = pods

    def play(self, pod_moves):
        
        for i, pod in enumerate(self.pods):
            target, thrust = pod_moves[i]
            pod.rotate(target)
            pod.boost(thrust)

        t = 0.0
        while t < 1.0:
            logging.debug(f"Start of loop: t = {t}")
            first_collision = None
            
            # Check collisions with checkpoints
            for pod in self.pods:
                checkpoint_to_check = self.checkpoints[pod.next_check_point_id]
                col = pod.collision(checkpoint_to_check)
                if col is not None and col.t + t < 1.0:
                    if first_collision is None or col.t < first_collision.t:
                        first_collision = col
            
            # Check collisions between pods
            for i in range(len(self.pods)):
                for j in range(i + 1, len(self.pods)):
                    col = self.pods[i].collision(self.pods[j])
                    if col is not None and col.t + t < 1.0:
                        if first_collision is None or col.t < first_collision.t:
                            first_collision = col

            if first_collision is None:
                logging.debug("No collision found. Moving to end of turn.")
                for pod in self.pods:
                    pod.move(1.0 - t)
                t = 1.0
            else:
                dt = first_collision.t
                logging.debug(f"Collision found: dt = {dt}, t = {t}")

                if dt > 0.0:
                    for pod in self.pods:
                        pod.move(dt)
                    t += dt

                first_collision.a.bounce(first_collision.b)

                if dt < 0.00001:
                    logging.warning("Very small dt detected, advancing time slightly to avoid loop.")
                    t += 0.001 # Epsilon to advance time
                
                logging.debug(f"End of collision handling: new t = {t}")


        for pod in self.pods:
            pod.end()

if __name__ == '__main__':
    checkpoints = [
        Checkpoint(8000, 4500, 0, CHECKPOINT_RADIUS),
        Checkpoint(16000, 0, 1, CHECKPOINT_RADIUS),
        Checkpoint(8000, -4500, 2, CHECKPOINT_RADIUS),
    ]

    pods = [
        Pod(0, 0, 0, POD_RADIUS, 0, 0, 0, 0),
        Pod(0, 1000, 1, POD_RADIUS, 0, 0, 0, 0),
    ]

    game = Game(checkpoints, pods)

    for i in range(10):
        print(f"Turn {i+1}")
        pod1 = game.pods[0]
        pod2 = game.pods[1]

        print(f"  Pod 1: x={pod1.x}, y={pod1.y}, vx={pod1.vx}, vy={pod1.vy}, angle={math.degrees(pod1.angle):.2f}, next_cp={pod1.next_check_point_id}")
        print(f"  Pod 2: x={pod2.x}, y={pod2.y}, vx={pod2.vx}, vy={pod2.vy}, angle={math.degrees(pod2.angle):.2f}, next_cp={pod2.next_check_point_id}")

        moves = [
            (checkpoints[pod1.next_check_point_id], 100),
            (checkpoints[pod2.next_check_point_id], 100),
        ]
        game.play(moves)