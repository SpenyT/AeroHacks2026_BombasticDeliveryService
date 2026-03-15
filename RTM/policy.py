from aerohacks.policy.base import Policy
from aerohacks.core.models import Observation, Plan, ActionStep, ActionType, Position2D
import math
import json
import os
import glob
import heapq

# =============================================================================
# STRATEGY v4-HYBRID: Cone Avoidance + Tangent Waypoints + v1 Layer Selection
# =============================================================================
# 1. Visibility graph with tangent-based waypoints + path smoothing (from candidate)
# 2. Cone-of-possible-positions traffic avoidance (from candidate)
# 3. ORIGINAL v1 detour-proportional layer selection (from best policy)
# 4. Reactive NOTAM integration: recompute path when NOTAMs appear/escalate
# 5. Restricted-zone consecutive step tracking with emergency escape
# 6. Energy-aware bailout to emergency landing sites
# =============================================================================

SPEED = 15.0
OBSTACLE_BUFFER = 30.0         # Buffer around static obstacles for visibility graph
CONSTRAINT_BUFFER = 25.0       # Buffer around constraints
TRAFFIC_DANGER_DIST = 120.0    # Distance to start CPA avoidance (kept for constraint push)
TRAFFIC_COLLISION_DIST = 40.0  # Emergency collision avoidance distance
WP_REACH_DIST = 50.0           # Waypoint reached threshold
MAP_MARGIN = 150.0             # Stay this far from map edges
REPLAN_INTERVAL = 10           # Ticks between full replans (was 25)
RESTRICTED_ESCAPE_THRESHOLD = 3  # Escape restricted zone before hitting 5

# --- Cone avoidance constants ---
COLLISION_DIST = 20.0       # Same layer = catastrophic
CONFLICT_DIST = 50.0        # Within +/-1 layer = -150/tick
ADVISORY_DIST = 100.0       # Within +/-1 layer = -10/tick
CONE_TURN_RATE = 0.04       # radians per tick of cone expansion
CONE_MAX_HALF_ANGLE = 0.8   # radians (~46 deg) cap
CONE_LOOKAHEAD = 20         # ticks to look ahead
CONE_TIME_STEPS = 10        # discrete time samples
CONE_DANGER_DIST = 130.0    # Start avoidance when cone edge is within this
CONE_EMERGENCY_DIST = 40.0  # Emergency push distance
CONE_AVOIDANCE_GAIN = 1.0   # Overall strength multiplier
WEIGHT_COLLISION = 20.0
WEIGHT_CONFLICT = 5.0
WEIGHT_ADVISORY = 1.0

EPSILON = 1e-9


# =============================================================================
# GEOMETRY UTILITIES
# =============================================================================

def dist(ax, ay, bx, by):
    return math.hypot(ax - bx, ay - by)


def cross2d(ox, oy, ax, ay, bx, by):
    """2D cross product of vectors OA and OB."""
    return (ax - ox) * (by - oy) - (ay - oy) * (bx - ox)


def point_in_polygon(px, py, verts):
    n = len(verts)
    inside = False
    j = n - 1
    for i in range(n):
        xi, yi = verts[i]
        xj, yj = verts[j]
        if ((yi > py) != (yj > py)) and (px < (xj - xi) * (py - yi) / (yj - yi) + xi):
            inside = not inside
        j = i
    return inside


def point_in_circle(px, py, cx, cy, r):
    return math.hypot(px - cx, py - cy) <= r


def seg_intersects_circle(x1, y1, x2, y2, cx, cy, r):
    """Check if line segment (x1,y1)-(x2,y2) intersects circle (cx,cy,r)."""
    dx, dy = x2 - x1, y2 - y1
    fx, fy = x1 - cx, y1 - cy
    a = dx * dx + dy * dy
    b = 2 * (fx * dx + fy * dy)
    c = fx * fx + fy * fy - r * r
    if a < 1e-12:
        return c <= 0
    disc = b * b - 4 * a * c
    if disc < 0:
        return False
    disc_sqrt = math.sqrt(disc)
    t1 = (-b - disc_sqrt) / (2 * a)
    t2 = (-b + disc_sqrt) / (2 * a)
    return t1 <= 1 and t2 >= 0


def seg_intersects_polygon(x1, y1, x2, y2, verts):
    """Check if line segment intersects a polygon (any edge or is inside)."""
    if point_in_polygon(x1, y1, verts) or point_in_polygon(x2, y2, verts):
        return True
    # Check multiple sample points along segment for long segments
    n_samples = max(3, int(math.hypot(x2 - x1, y2 - y1) / 500))
    for s in range(1, n_samples):
        t = s / n_samples
        sx = x1 + t * (x2 - x1)
        sy = y1 + t * (y2 - y1)
        if point_in_polygon(sx, sy, verts):
            return True
    n = len(verts)
    for i in range(n):
        j = (i + 1) % n
        if segments_intersect(x1, y1, x2, y2, verts[i][0], verts[i][1], verts[j][0], verts[j][1]):
            return True
    return False


def segments_intersect(ax1, ay1, ax2, ay2, bx1, by1, bx2, by2):
    """Check if two line segments intersect."""
    d1 = cross2d(bx1, by1, bx2, by2, ax1, ay1)
    d2 = cross2d(bx1, by1, bx2, by2, ax2, ay2)
    d3 = cross2d(ax1, ay1, ax2, ay2, bx1, by1)
    d4 = cross2d(ax1, ay1, ax2, ay2, bx2, by2)

    if ((d1 > 0 and d2 < 0) or (d1 < 0 and d2 > 0)) and \
       ((d3 > 0 and d4 < 0) or (d3 < 0 and d4 > 0)):
        return True
    return False


def expand_polygon(verts, buffer):
    """Expand a polygon outward by buffer distance. Returns expanded vertices."""
    n = len(verts)
    if n < 3:
        return verts

    cx = sum(v[0] for v in verts) / n
    cy = sum(v[1] for v in verts) / n

    expanded = []
    for i in range(n):
        vx, vy = verts[i]
        dx = vx - cx
        dy = vy - cy
        mag = math.hypot(dx, dy)
        if mag < 1e-6:
            expanded.append((vx, vy))
            continue
        expanded.append((vx + dx / mag * buffer, vy + dy / mag * buffer))
    return expanded


def get_polygon_corners_with_buffer(verts, buffer):
    """Get the corner waypoints of a polygon expanded by buffer."""
    return expand_polygon(verts, buffer)


def dist_to_seg(px, py, x1, y1, x2, y2):
    dx, dy = x2 - x1, y2 - y1
    len2 = dx * dx + dy * dy
    if len2 < 1e-12:
        return math.hypot(px - x1, py - y1)
    t = max(0.0, min(1.0, ((px - x1) * dx + (py - y1) * dy) / len2))
    return math.hypot(px - (x1 + t * dx), py - (y1 + t * dy))


# =============================================================================
# TANGENT COMPUTATION (from tsp_optimization research)
# =============================================================================

def circle_tangent_points(px, py, cx, cy, r):
    """Compute the two external tangent points on a circle from an external point P."""
    d = dist(px, py, cx, cy)
    if d <= r + EPSILON:
        return []
    theta = math.atan2(py - cy, px - cx)
    alpha = math.acos(r / d)
    t1 = (cx + r * math.cos(theta + alpha), cy + r * math.sin(theta + alpha))
    t2 = (cx + r * math.cos(theta - alpha), cy + r * math.sin(theta - alpha))
    return [t1, t2]


def circle_tangent_waypoints(px, py, cx, cy, r_buffered):
    """Get tangent-based waypoints for navigating around a circle obstacle."""
    return circle_tangent_points(px, py, cx, cy, r_buffered)


def polygon_silhouette_vertices(px, py, verts):
    """Find the silhouette (tangent) vertices of a convex polygon as seen from point P."""
    n = len(verts)
    if n < 3:
        return list(verts)

    best_left = 0
    best_right = 0

    for i in range(1, n):
        cp_left = cross2d(px, py, verts[best_left][0], verts[best_left][1],
                          verts[i][0], verts[i][1])
        if cp_left > 0:
            best_left = i

        cp_right = cross2d(px, py, verts[best_right][0], verts[best_right][1],
                           verts[i][0], verts[i][1])
        if cp_right < 0:
            best_right = i

    result = [verts[best_left]]
    if best_right != best_left:
        result.append(verts[best_right])

    return result


def polygon_tangent_waypoints(px, py, verts, buffer):
    """Get tangent-based waypoints for navigating around a polygon obstacle."""
    expanded = expand_polygon(verts, buffer)
    return polygon_silhouette_vertices(px, py, expanded)


# =============================================================================
# CONE AVOIDANCE (from cone_avoidance research)
# =============================================================================

def _angle_of(x, y):
    """Return angle in radians [-pi, pi] of vector (x, y)."""
    return math.atan2(y, x)


def _angle_diff(a, b):
    """Signed angular difference a - b, normalized to [-pi, pi]."""
    d = a - b
    while d > math.pi:
        d -= 2 * math.pi
    while d < -math.pi:
        d += 2 * math.pi
    return d


def _rotate(x, y, angle):
    """Rotate vector (x, y) by angle radians."""
    c = math.cos(angle)
    s = math.sin(angle)
    return x * c - y * s, x * s + y * c


class NPCCone:
    """Models the set of possible future positions of one NPC as an expanding cone."""

    def __init__(self, npc_x, npc_y, npc_vx, npc_vy, npc_alt):
        self.ox = npc_x
        self.oy = npc_y
        self.vx = npc_vx
        self.vy = npc_vy
        self.alt = npc_alt
        self.speed = math.hypot(npc_vx, npc_vy)
        self.heading = _angle_of(npc_vx, npc_vy) if self.speed > 0.5 else 0.0

    def center_at(self, t):
        return self.ox + self.vx * t, self.oy + self.vy * t

    def half_angle_at(self, t):
        return min(CONE_TURN_RATE * t, CONE_MAX_HALF_ANGLE)

    def arc_radius_at(self, t):
        return self.speed * t

    def distance_to_cone_edge(self, px, py, t):
        cx, cy = self.center_at(t)
        half_angle = self.half_angle_at(t)
        dx = px - cx
        dy = py - cy
        d = math.hypot(dx, dy)

        if d < 1e-6:
            return -half_angle * self.arc_radius_at(t), 0

        if self.speed < 0.5:
            return d, 0

        point_angle = _angle_of(dx, dy)
        angle_from_axis = _angle_diff(point_angle, self.heading)
        side = 1 if angle_from_axis >= 0 else -1
        abs_angle = abs(angle_from_axis)

        if abs_angle <= half_angle:
            edge_angle = half_angle * side
            edge_dir_x, edge_dir_y = _rotate(
                math.cos(self.heading), math.sin(self.heading), edge_angle
            )
            cross_val = dx * edge_dir_y - dy * edge_dir_x
            perp_dist = abs(cross_val)
            return -perp_dist, side
        else:
            nearest_edge_angle = half_angle if angle_from_axis > 0 else -half_angle
            edge_global_angle = self.heading + nearest_edge_angle
            edge_dx = math.cos(edge_global_angle)
            edge_dy = math.sin(edge_global_angle)
            dot = dx * edge_dx + dy * edge_dy
            if dot < 0:
                return d, side
            else:
                proj_x = edge_dx * dot
                proj_y = edge_dy * dot
                perp_dist = math.hypot(dx - proj_x, dy - proj_y)
                return perp_dist, side

    def best_avoidance_direction(self, px, py, t):
        cx, cy = self.center_at(t)
        half_angle = self.half_angle_at(t)
        dx = px - cx
        dy = py - cy
        d = math.hypot(dx, dy)

        if d < 1e-6:
            perp_x = -math.sin(self.heading)
            perp_y = math.cos(self.heading)
            return perp_x, perp_y, 1.0

        if self.speed < 0.5:
            return dx / d, dy / d, 1.0

        point_angle = _angle_of(dx, dy)
        angle_from_axis = _angle_diff(point_angle, self.heading)
        abs_angle = abs(angle_from_axis)

        if abs_angle <= half_angle:
            dist_to_left_edge = half_angle - angle_from_axis
            dist_to_right_edge = half_angle + angle_from_axis
            if dist_to_left_edge < dist_to_right_edge:
                escape_angle = self.heading + half_angle + 0.15
            else:
                escape_angle = self.heading - half_angle - 0.15
            esc_x = math.cos(escape_angle)
            esc_y = math.sin(escape_angle)
            return esc_x, esc_y, 1.0
        else:
            if angle_from_axis > 0:
                nearest_edge_angle = self.heading + half_angle
            else:
                nearest_edge_angle = self.heading - half_angle
            edge_dx = math.cos(nearest_edge_angle)
            edge_dy = math.sin(nearest_edge_angle)
            if angle_from_axis > 0:
                out_x = -edge_dy
                out_y = edge_dx
            else:
                out_x = edge_dy
                out_y = -edge_dx
            return out_x, out_y, 0.5


def _compute_cone_threat(px, py, pvx, pvy, cone, current_alt, lookahead):
    """For a single NPC cone, compute threat level across prediction window."""
    alt_diff = abs(current_alt - cone.alt)
    if alt_diff > 1:
        return 0.0, 0.0, 0

    same_layer = (current_alt == cone.alt)
    dt = lookahead / CONE_TIME_STEPS
    best_avx, best_avy = 0.0, 0.0
    max_threat = 0
    max_urgency = 0.0

    for i in range(1, CONE_TIME_STEPS + 1):
        t = i * dt
        drone_x = px + pvx * t
        drone_y = py + pvy * t
        cx, cy = cone.center_at(t)
        center_dist = dist(drone_x, drone_y, cx, cy)

        max_cone_radius = cone.speed * t * math.sin(cone.half_angle_at(t)) if cone.speed > 0.5 else 0
        if center_dist > CONE_DANGER_DIST + max_cone_radius + 50:
            continue

        cone_edge_dist, side = cone.distance_to_cone_edge(drone_x, drone_y, t)

        if cone_edge_dist < 0:
            eff_dist = center_dist
        else:
            eff_dist = min(center_dist, center_dist + cone_edge_dist)

        threat = 0
        if same_layer and eff_dist <= COLLISION_DIST:
            threat = 3
        elif eff_dist <= CONFLICT_DIST:
            threat = 2
        elif eff_dist <= ADVISORY_DIST:
            threat = 1

        in_danger = eff_dist < CONE_DANGER_DIST or cone_edge_dist < 0

        if not in_danger and threat == 0:
            continue

        avx_t, avy_t, dir_urgency = cone.best_avoidance_direction(drone_x, drone_y, t)
        time_urgency = max(0.0, 1.0 - t / lookahead)

        if eff_dist < CONE_EMERGENCY_DIST:
            dist_urgency = 2.0
        elif eff_dist < CONFLICT_DIST:
            dist_urgency = 1.5
        elif eff_dist < ADVISORY_DIST:
            dist_urgency = 1.0
        else:
            dist_urgency = max(0.0, (CONE_DANGER_DIST - eff_dist) / CONE_DANGER_DIST)

        if same_layer and eff_dist < COLLISION_DIST * 2:
            penalty_weight = WEIGHT_COLLISION
        elif eff_dist < CONFLICT_DIST * 1.5:
            penalty_weight = WEIGHT_CONFLICT
        elif eff_dist < ADVISORY_DIST * 1.2:
            penalty_weight = WEIGHT_ADVISORY
        else:
            penalty_weight = 0.5

        if same_layer:
            penalty_weight *= 1.5

        if cone_edge_dist < 0:
            penalty_weight *= 1.5

        total_urgency = time_urgency * dist_urgency * dir_urgency * penalty_weight

        if total_urgency > max_urgency:
            max_urgency = total_urgency
            best_avx = avx_t * total_urgency * SPEED * CONE_AVOIDANCE_GAIN
            best_avy = avy_t * total_urgency * SPEED * CONE_AVOIDANCE_GAIN

        if threat > max_threat:
            max_threat = threat

    return best_avx, best_avy, max_threat


def _emergency_avoidance(px, py, npc_x, npc_y, cur_dist):
    """When an NPC is very close, use simple reactive push."""
    if cur_dist < 1e-6:
        return SPEED * 3.0, 0.0
    strength = (CONE_EMERGENCY_DIST - cur_dist) / CONE_EMERGENCY_DIST * SPEED * 3.0
    avx = (px - npc_x) / cur_dist * strength
    avy = (py - npc_y) / cur_dist * strength
    return avx, avy


def cone_avoidance_vector(px, py, pvx, pvy, traffic, current_alt):
    """Compute avoidance vector using cone-of-possible-positions for all traffic.
    Drop-in replacement for cpa_avoidance_vector()."""
    total_avx, total_avy = 0.0, 0.0
    threats = []

    for t in traffic:
        try:
            npc_x = t.position.x
            npc_y = t.position.y
            npc_alt = t.alt_layer

            if abs(current_alt - npc_alt) > 1:
                continue

            npc_vx = t.velocity.x if t.velocity else 0.0
            npc_vy = t.velocity.y if t.velocity else 0.0

            cur_dist = dist(px, py, npc_x, npc_y)

            if cur_dist < CONE_EMERGENCY_DIST:
                eavx, eavy = _emergency_avoidance(px, py, npc_x, npc_y, cur_dist)
                threats.append((3, eavx, eavy, cur_dist))
                continue

            cone = NPCCone(npc_x, npc_y, npc_vx, npc_vy, npc_alt)
            avx, avy, threat_level = _compute_cone_threat(
                px, py, pvx, pvy, cone, current_alt, CONE_LOOKAHEAD
            )

            if abs(avx) > 1e-6 or abs(avy) > 1e-6:
                threats.append((threat_level, avx, avy, cur_dist))

        except Exception:
            pass

    if not threats:
        return 0.0, 0.0

    for threat_level, avx, avy, cur_dist in threats:
        if threat_level >= 3:
            priority = 3.0
        elif threat_level >= 2:
            priority = 1.5
        elif threat_level >= 1:
            priority = 0.8
        else:
            priority = 0.5
        total_avx += avx * priority
        total_avy += avy * priority

    return total_avx, total_avy


# =============================================================================
# OBSTACLE REPRESENTATION (enhanced with tangent waypoints + bounding box)
# =============================================================================

class Obstacle:
    """Represents any obstacle or constraint region."""

    def __init__(self, kind, data, alt_layers=None, is_static=False, phase=None):
        self.kind = kind  # 'circle' or 'polygon'
        self.data = data  # (cx, cy, r) for circle, [(x,y),...] for polygon
        self.alt_layers = alt_layers or []
        self.is_static = is_static
        self.phase = phase

        # Pre-compute bounding box for fast spatial filtering
        if kind == 'circle':
            cx, cy, r = data
            self.bbox = (cx - r, cy - r, cx + r, cy + r)
            self._cx, self._cy, self._r = cx, cy, r
        else:
            xs = [v[0] for v in data]
            ys = [v[1] for v in data]
            self.bbox = (min(xs), min(ys), max(xs), max(ys))
            self._cx = sum(xs) / len(xs)
            self._cy = sum(ys) / len(ys)
            self._r = max(dist(self._cx, self._cy, v[0], v[1]) for v in data)

    def blocks_layer(self, layer):
        if self.is_static or not self.alt_layers:
            return True
        return layer in self.alt_layers

    def contains(self, px, py):
        if self.kind == 'circle':
            cx, cy, r = self.data
            return point_in_circle(px, py, cx, cy, r)
        return point_in_polygon(px, py, self.data)

    def signed_dist(self, px, py):
        if self.kind == 'circle':
            cx, cy, r = self.data
            return math.hypot(px - cx, py - cy) - r
        verts = self.data
        min_d = float('inf')
        n = len(verts)
        for i in range(n):
            j = (i + 1) % n
            dd = dist_to_seg(px, py, verts[i][0], verts[i][1], verts[j][0], verts[j][1])
            min_d = min(min_d, dd)
        return -min_d if point_in_polygon(px, py, verts) else min_d

    def centroid(self):
        return self._cx, self._cy

    def bounding_radius(self):
        return self._r

    def get_waypoints(self, buffer):
        """Get navigation waypoints around this obstacle (fallback 4-point for circles)."""
        if self.kind == 'circle':
            cx, cy, r = self.data
            rb = r + buffer
            return [
                (cx + rb, cy), (cx - rb, cy),
                (cx, cy + rb), (cx, cy - rb),
            ]
        return get_polygon_corners_with_buffer(self.data, buffer)

    def get_tangent_waypoints(self, px, py, buffer):
        """Get tangent-based waypoints from a specific source point."""
        if self.kind == 'circle':
            cx, cy, r = self.data
            return circle_tangent_waypoints(px, py, cx, cy, r + buffer)
        return polygon_tangent_waypoints(px, py, self.data, buffer)

    def get_all_tangent_waypoints(self, sources, buffer):
        """Get tangent waypoints from multiple source points, deduplicated."""
        all_wps = set()
        for sx, sy in sources:
            for wp in self.get_tangent_waypoints(sx, sy, buffer):
                all_wps.add((round(wp[0], 2), round(wp[1], 2)))
        return list(all_wps)

    def seg_intersects(self, x1, y1, x2, y2, buffer=0):
        """Check if a line segment intersects this obstacle (with bounding-box pre-check)."""
        # Fast bounding-box rejection
        seg_min_x = min(x1, x2) - buffer
        seg_max_x = max(x1, x2) + buffer
        seg_min_y = min(y1, y2) - buffer
        seg_max_y = max(y1, y2) + buffer
        bx0, by0, bx1, by1 = self.bbox
        if seg_max_x < bx0 - buffer or seg_min_x > bx1 + buffer:
            return False
        if seg_max_y < by0 - buffer or seg_min_y > by1 + buffer:
            return False

        if self.kind == 'circle':
            cx, cy, r = self.data
            return seg_intersects_circle(x1, y1, x2, y2, cx, cy, r + buffer)
        if buffer > 0:
            expanded = expand_polygon(self.data, buffer)
        else:
            expanded = self.data
        return seg_intersects_polygon(x1, y1, x2, y2, expanded)


# =============================================================================
# CORRIDOR RELEVANCE FILTER (from tsp_optimization research)
# =============================================================================

def obstacle_relevant_to_path(obs, start, goal, corridor_half_width):
    """Check if an obstacle is potentially relevant to the start-goal path."""
    sx, sy = start
    gx, gy = goal
    cx, cy = obs.centroid()
    r = obs.bounding_radius()

    dx, dy = gx - sx, gy - sy
    seg_len = math.hypot(dx, dy)
    if seg_len < EPSILON:
        return dist(sx, sy, cx, cy) < r + corridor_half_width

    t = ((cx - sx) * dx + (cy - sy) * dy) / (seg_len * seg_len)
    t = max(-0.1, min(1.1, t))
    proj_x = sx + t * dx
    proj_y = sy + t * dy

    d = dist(cx, cy, proj_x, proj_y)
    return d < r + corridor_half_width


# =============================================================================
# PATH SMOOTHING (from tsp_optimization research)
# =============================================================================

def smooth_path(path, obstacles):
    """Attempt to remove unnecessary intermediate waypoints from the path."""
    if len(path) <= 2:
        return path

    smoothed = [path[0]]
    i = 0
    while i < len(path) - 1:
        best_j = i + 1
        for j in range(len(path) - 1, i + 1, -1):
            x1, y1 = path[i]
            x2, y2 = path[j]
            clear = True
            for obs in obstacles:
                buf = OBSTACLE_BUFFER * 0.7 if obs.is_static else CONSTRAINT_BUFFER * 0.5
                if obs.seg_intersects(x1, y1, x2, y2, buf):
                    clear = False
                    break
            if clear:
                best_j = j
                break
        smoothed.append(path[best_j])
        i = best_j

    return smoothed


# =============================================================================
# VISIBILITY GRAPH + DIJKSTRA (enhanced with tangent waypoints + corridor filter)
# =============================================================================

def build_visibility_graph(start, goal, obstacles, layer, map_bounds, corridor_width=8000.0):
    """Build a visibility graph using tangent-based waypoints and find shortest path."""
    min_x, max_x, min_y, max_y = map_bounds

    # Step 1: Filter obstacles to those relevant to this layer and corridor
    active_obstacles = []
    for obs in obstacles:
        if not obs.blocks_layer(layer):
            continue
        if obstacle_relevant_to_path(obs, start, goal, corridor_width):
            active_obstacles.append(obs)

    # If no obstacles block the path, just return direct line
    if not active_obstacles:
        return [start, goal]

    # Quick check: is the direct path clear?
    direct_clear = True
    for obs in active_obstacles:
        buf = OBSTACLE_BUFFER * 0.7 if obs.is_static else CONSTRAINT_BUFFER * 0.5
        if obs.seg_intersects(start[0], start[1], goal[0], goal[1], buf):
            direct_clear = False
            break
    if direct_clear:
        return [start, goal]

    # Step 2: Generate tangent-based waypoints
    nodes = [start, goal]
    node_set = {(round(start[0], 1), round(start[1], 1)),
                (round(goal[0], 1), round(goal[1], 1))}

    # Phase 1: tangent points from start and goal to each obstacle
    key_sources = [start, goal]
    for obs in active_obstacles:
        buffer = OBSTACLE_BUFFER if obs.is_static else CONSTRAINT_BUFFER
        wps = obs.get_all_tangent_waypoints(key_sources, buffer)
        for wp in wps:
            wp_key = (round(wp[0], 1), round(wp[1], 1))
            if wp_key in node_set:
                continue
            if not (min_x + MAP_MARGIN <= wp[0] <= max_x - MAP_MARGIN and
                    min_y + MAP_MARGIN <= wp[1] <= max_y - MAP_MARGIN):
                continue
            inside = False
            for other in active_obstacles:
                if other.contains(wp[0], wp[1]):
                    inside = True
                    break
            if not inside:
                nodes.append(wp)
                node_set.add(wp_key)

    # Phase 2: inter-obstacle tangent points
    if len(active_obstacles) > 1:
        obs_waypoints = {}
        for i, obs in enumerate(active_obstacles):
            buffer = OBSTACLE_BUFFER if obs.is_static else CONSTRAINT_BUFFER
            nearby = [n for n in nodes[2:] if dist(n[0], n[1], obs._cx, obs._cy) < obs._r + buffer + 500]
            if not nearby:
                nearby = key_sources
            obs_waypoints[i] = nearby

        for i, obs_i in enumerate(active_obstacles):
            for j, obs_j in enumerate(active_obstacles):
                if i >= j:
                    continue
                inter_dist = dist(obs_i._cx, obs_i._cy, obs_j._cx, obs_j._cy)
                if inter_dist > obs_i._r + obs_j._r + 2 * OBSTACLE_BUFFER + corridor_width:
                    continue
                buffer_j = OBSTACLE_BUFFER if obs_j.is_static else CONSTRAINT_BUFFER
                for src in obs_waypoints.get(i, [])[:4]:
                    new_wps = obs_j.get_tangent_waypoints(src[0], src[1], buffer_j)
                    for wp in new_wps:
                        wp_key = (round(wp[0], 1), round(wp[1], 1))
                        if wp_key in node_set:
                            continue
                        if not (min_x + MAP_MARGIN <= wp[0] <= max_x - MAP_MARGIN and
                                min_y + MAP_MARGIN <= wp[1] <= max_y - MAP_MARGIN):
                            continue
                        inside = False
                        for other in active_obstacles:
                            if other.contains(wp[0], wp[1]):
                                inside = True
                                break
                        if not inside:
                            nodes.append(wp)
                            node_set.add(wp_key)

    # Phase 3: Fallback waypoints for obstacles with no tangent points nearby
    for obs in active_obstacles:
        buffer = OBSTACLE_BUFFER if obs.is_static else CONSTRAINT_BUFFER
        has_nearby = any(
            dist(n[0], n[1], obs._cx, obs._cy) < obs._r + buffer + 200
            for n in nodes[2:]
        )
        if not has_nearby:
            fallback = obs.get_waypoints(buffer)
            for wp in fallback:
                wp_key = (round(wp[0], 1), round(wp[1], 1))
                if wp_key in node_set:
                    continue
                if not (min_x + MAP_MARGIN <= wp[0] <= max_x - MAP_MARGIN and
                        min_y + MAP_MARGIN <= wp[1] <= max_y - MAP_MARGIN):
                    continue
                inside = False
                for other in active_obstacles:
                    if other.contains(wp[0], wp[1]):
                        inside = True
                        break
                if not inside:
                    nodes.append(wp)
                    node_set.add(wp_key)

    # Step 3: Build adjacency list (visibility check with bounding-box pruning)
    n = len(nodes)
    adj = [[] for _ in range(n)]

    direct_dist = dist(start[0], start[1], goal[0], goal[1])

    for i in range(n):
        for j in range(i + 1, n):
            x1, y1 = nodes[i]
            x2, y2 = nodes[j]

            edge_len = dist(x1, y1, x2, y2)
            if edge_len > direct_dist * 1.5:
                continue

            mx, my = (x1 + x2) / 2, (y1 + y2) / 2
            if not (min_x + MAP_MARGIN / 2 <= mx <= max_x - MAP_MARGIN / 2 and
                    min_y + MAP_MARGIN / 2 <= my <= max_y - MAP_MARGIN / 2):
                continue

            blocked = False
            for obs in active_obstacles:
                if obs.is_static:
                    buf = OBSTACLE_BUFFER * 0.7
                elif obs.phase and 'RESTRICTED' in str(obs.phase):
                    buf = CONSTRAINT_BUFFER * 0.7
                else:
                    buf = CONSTRAINT_BUFFER * 0.5
                if obs.seg_intersects(x1, y1, x2, y2, buf):
                    blocked = True
                    break
            if not blocked:
                adj[i].append((j, edge_len))
                adj[j].append((i, edge_len))

    # Step 4: Dijkstra shortest path
    INF = float('inf')
    dists_arr = [INF] * n
    dists_arr[0] = 0
    prev = [-1] * n
    pq = [(0.0, 0)]

    while pq:
        cost, u = heapq.heappop(pq)
        if cost > dists_arr[u]:
            continue
        if u == 1:
            break
        for v, w in adj[u]:
            new_cost = cost + w
            if new_cost < dists_arr[v]:
                dists_arr[v] = new_cost
                prev[v] = u
                heapq.heappush(pq, (new_cost, v))

    if dists_arr[1] == INF:
        return [start, goal]

    path = []
    cur = 1
    while cur != -1:
        path.append(nodes[cur])
        cur = prev[cur]
    path.reverse()

    # Step 5: Path smoothing
    path = smooth_path(path, active_obstacles)

    return path


# =============================================================================
# SCENARIO LOADING
# =============================================================================

def load_matching_scenario(obs):
    """Find the public scenario JSON matching current observation."""
    import sys

    base_dirs = [
        os.path.join(os.path.dirname(__file__), "..", "scenarios", "public"),
        "scenarios/public",
    ]

    scenario_hint = None
    for i, arg in enumerate(sys.argv):
        if arg == '--scenario' and i + 1 < len(sys.argv):
            scenario_hint = sys.argv[i + 1]
            break

    if scenario_hint:
        for base_dir in base_dirs:
            for candidate in [
                os.path.join(base_dir, scenario_hint + ".json"),
                os.path.join(base_dir, scenario_hint),
                scenario_hint,
            ]:
                if os.path.exists(candidate):
                    try:
                        with open(candidate, "r") as f:
                            data = json.load(f)
                        return data
                    except Exception:
                        pass
            try:
                files = glob.glob(os.path.join(base_dir, "*.json"))
                for path in files:
                    try:
                        with open(path, "r") as f:
                            data = json.load(f)
                        sid = data.get("scenario_id", "")
                        if sid and sid in scenario_hint:
                            return data
                    except Exception:
                        continue
            except Exception:
                pass

    sx = obs.ownship_state.position.x
    sy = obs.ownship_state.position.y

    obs_constraint_ids = set()
    try:
        for c in obs.active_constraints:
            try:
                obs_constraint_ids.add(c.id)
            except Exception:
                pass
    except Exception:
        pass

    goal_verts = []
    try:
        goal_verts = [(v.x, v.y) for v in obs.mission_goal.region.vertices]
    except Exception:
        pass

    candidates = []
    for base_dir in base_dirs:
        try:
            files = glob.glob(os.path.join(base_dir, "*.json"))
        except Exception:
            continue
        for path in files:
            try:
                with open(path, "r") as f:
                    data = json.load(f)
                ss = data.get("start_state", {})
                pos = ss.get("position", {})
                if abs(pos.get("x", -1) - sx) < 1 and abs(pos.get("y", -1) - sy) < 1:
                    score = 0
                    if goal_verts:
                        mg = data.get("mission_goal", {}).get("region", {})
                        mg_verts = mg.get("vertices", [])
                        if mg_verts:
                            file_goal = [(v["x"], v["y"]) for v in mg_verts]
                            if len(file_goal) == len(goal_verts) and all(
                                abs(fv[0] - gv[0]) < 10 and abs(fv[1] - gv[1]) < 10
                                for fv, gv in zip(file_goal, goal_verts)
                            ):
                                score += 10
                    if obs_constraint_ids:
                        perm_ids = {c.get("id", "") for c in data.get("permanent_constraints", [])}
                        score += len(obs_constraint_ids & perm_ids) * 5
                    candidates.append((score, path, data))
            except Exception:
                continue

    if candidates:
        candidates.sort(key=lambda x: -x[0])
        best = candidates[0]
        return best[2]

    return None


def parse_obstacles(scenario):
    """Parse all static obstacles and permanent constraints from scenario."""
    obstacles = []

    for s in scenario.get("static_obstacles", []):
        if s.get("type") == "CircleRegion":
            obstacles.append(Obstacle(
                'circle',
                (s["center_pos"]["x"], s["center_pos"]["y"], s["radius"]),
                is_static=True
            ))
        else:
            verts = [(v["x"], v["y"]) for v in s.get("vertices", [])]
            if verts:
                obstacles.append(Obstacle('polygon', verts, is_static=True))

    for c in scenario.get("permanent_constraints", []):
        r = c["region"]
        alts = c.get("alt_layers", [1, 2, 3, 4])
        if r.get("type") == "CircleRegion":
            obstacles.append(Obstacle(
                'circle',
                (r["center_pos"]["x"], r["center_pos"]["y"], r["radius"]),
                alt_layers=alts
            ))
        else:
            verts = [(v["x"], v["y"]) for v in r.get("vertices", [])]
            if verts:
                obstacles.append(Obstacle('polygon', verts, alt_layers=alts))

    return obstacles


def parse_emergency_sites(scenario):
    """Parse emergency landing sites."""
    sites = []
    for s in scenario.get("emergency_landing_sites", []):
        verts = s.get("region", {}).get("vertices", [])
        if verts:
            cx = sum(v["x"] for v in verts) / len(verts)
            cy = sum(v["y"] for v in verts) / len(verts)
            sites.append((cx, cy))
    return sites


def get_map_bounds(scenario):
    mb = scenario.get("map_boundaries", {})
    verts = mb.get("vertices", [])
    if verts:
        xs = [v["x"] for v in verts]
        ys = [v["y"] for v in verts]
        return min(xs), max(xs), min(ys), max(ys)
    return 0, 40000, 0, 40000


# =============================================================================
# MAIN POLICY
# =============================================================================

class MyPolicy(Policy):

    def __init__(self):
        self.initialized = False
        self.scenario = None
        self.obstacles = []            # Static + permanent constraints
        self.emergency_sites = []
        self.map_bounds = (0, 40000, 0, 40000)
        self.goal = None
        self.goal_verts = []           # Goal polygon vertices [(x,y), ...]
        self.target_alt = 1
        self.energy_decay = 0.1
        self.max_time = 5000

        # Path state
        self.path = []                 # Current visibility-graph path [(x,y), ...]
        self.path_idx = 0              # Current waypoint index in path
        self.current_layer = 1         # Current altitude layer we're flying at

        # Dynamic obstacle tracking
        self.known_notams = {}         # id -> Obstacle
        self.last_notam_count = 0

        # Predictive NOTAM tracking (observation-based)
        self.last_notam_phases = {}    # id -> last observed phase string
        self.last_constraint_ids = set()  # Track ALL constraint IDs (incl ADVISORY)
        self.notam_history = {}        # id -> {'advisory_tick': T, 'controlled_tick': T, ...}

        # NPC traffic tracking
        self.last_traffic_ids = set()  # Track known NPC IDs

        # Restricted zone tracking
        self.restricted_consecutive = 0

        # Tick counter
        self.tick = 0
        self.last_replan_tick = -999

    def _init(self, obs):
        """One-time initialization from first observation."""
        self.initialized = True

        self.scenario = load_matching_scenario(obs)

        start = (obs.ownship_state.position.x, obs.ownship_state.position.y)
        self.goal, self.goal_verts = self._extract_goal(obs, start)
        self.target_alt = obs.mission_goal.target_alt_layer if obs.mission_goal.target_alt_layer is not None else 1

        if self.scenario:
            self.obstacles = parse_obstacles(self.scenario)
            self.map_bounds = get_map_bounds(self.scenario)
            self.emergency_sites = parse_emergency_sites(self.scenario)
            vl = self.scenario.get("vehicle_limits", {})
            self.energy_decay = vl.get("energy_decay_rate", 0.1)
            sc = self.scenario.get("scoring_config", {})
            self.max_time = sc.get("max_time", 5000)

        # Initial layer selection
        self.current_layer = self._pick_best_layer(start, obs)

        # Build initial path
        self._replan(start, obs)

    def _extract_goal(self, obs, start):
        """Extract goal region and target the nearest corner for fastest arrival."""
        goal_verts_tuples = []

        try:
            verts = obs.mission_goal.region.vertices
            goal_verts_tuples = [(v.x, v.y) for v in verts]
        except Exception:
            pass

        if not goal_verts_tuples and self.scenario:
            mr = self.scenario.get("mission_goal", {}).get("region", {})
            verts = mr.get("vertices", [])
            if verts:
                goal_verts_tuples = [(v["x"], v["y"]) for v in verts]

        if goal_verts_tuples:
            candidates = list(goal_verts_tuples)
            for i in range(len(goal_verts_tuples)):
                j = (i + 1) % len(goal_verts_tuples)
                mx = (goal_verts_tuples[i][0] + goal_verts_tuples[j][0]) / 2
                my = (goal_verts_tuples[i][1] + goal_verts_tuples[j][1]) / 2
                candidates.append((mx, my))

            best_point = min(candidates, key=lambda p: dist(start[0], start[1], p[0], p[1]))
            return best_point, goal_verts_tuples

        gx, gy = None, None
        try:
            c = obs.mission_goal.region.center()
            gx, gy = c.x, c.y
        except Exception:
            pass
        if gx is None:
            try:
                gx = obs.mission_goal.region.center_pos.x
                gy = obs.mission_goal.region.center_pos.y
            except Exception:
                pass
        if gx is None:
            gx, gy = 37000.0, 36000.0
        return (gx, gy), []

    def _adjust_goal_for_notams(self, obs):
        """If current goal point is inside a RESTRICTED/CONTROLLED NOTAM on
        the target layer, switch to a goal point outside the NOTAM."""
        if not self.goal_verts:
            return

        for c in obs.active_constraints:
            try:
                phase = str(c.phase)
                if 'RESTRICTED' not in phase and 'CONTROLLED' not in phase:
                    continue
                alts = list(c.alt_layers) if hasattr(c, 'alt_layers') and c.alt_layers else []
                if self.target_alt not in alts:
                    continue

                region = c.region
                inside_goal = False
                check_fn = None
                try:
                    cx, cy, r = region.center_pos.x, region.center_pos.y, region.radius
                    inside_goal = point_in_circle(self.goal[0], self.goal[1], cx, cy, r)
                    check_fn = lambda px, py: point_in_circle(px, py, cx, cy, r)
                except Exception:
                    try:
                        verts = [(v.x, v.y) for v in region.vertices]
                        inside_goal = point_in_polygon(self.goal[0], self.goal[1], verts)
                        check_fn = lambda px, py: point_in_polygon(px, py, verts)
                    except Exception:
                        continue

                if inside_goal and check_fn:
                    candidates = list(self.goal_verts)
                    for i in range(len(self.goal_verts)):
                        j = (i + 1) % len(self.goal_verts)
                        mx = (self.goal_verts[i][0] + self.goal_verts[j][0]) / 2
                        my = (self.goal_verts[i][1] + self.goal_verts[j][1]) / 2
                        candidates.append((mx, my))
                    center_x = sum(v[0] for v in self.goal_verts) / len(self.goal_verts)
                    center_y = sum(v[1] for v in self.goal_verts) / len(self.goal_verts)
                    candidates.append((center_x, center_y))
                    for v in self.goal_verts:
                        candidates.append((v[0] + (center_x - v[0]) * 0.1,
                                           v[1] + (center_y - v[1]) * 0.1))

                    safe = [p for p in candidates if not check_fn(p[0], p[1])]
                    if safe:
                        self.goal = max(safe, key=lambda p: dist(
                            p[0], p[1], cx, cy))
                        return
            except Exception:
                pass

    def _point_in_goal(self, px, py):
        """Check if a point is inside the goal region."""
        if self.goal_verts:
            return point_in_polygon(px, py, self.goal_verts)
        return dist(px, py, self.goal[0], self.goal[1]) < 100

    def _constraint_overlaps_goal(self, region_data, kind):
        """Check if a constraint region overlaps with the goal region."""
        if not self.goal_verts:
            return False

        if kind == 'circle':
            cx, cy, r = region_data
            for vx, vy in self.goal_verts:
                if dist(vx, vy, cx, cy) <= r:
                    return True
            if point_in_polygon(cx, cy, self.goal_verts):
                return True
            n = len(self.goal_verts)
            for i in range(n):
                j = (i + 1) % n
                if seg_intersects_circle(
                    self.goal_verts[i][0], self.goal_verts[i][1],
                    self.goal_verts[j][0], self.goal_verts[j][1],
                    cx, cy, r
                ):
                    return True
            return False
        else:
            poly = region_data
            for vx, vy in poly:
                if point_in_polygon(vx, vy, self.goal_verts):
                    return True
            for vx, vy in self.goal_verts:
                if point_in_polygon(vx, vy, poly):
                    return True
            n1 = len(poly)
            n2 = len(self.goal_verts)
            for i in range(n1):
                j = (i + 1) % n1
                for k in range(n2):
                    l_idx = (k + 1) % n2
                    if segments_intersect(
                        poly[i][0], poly[i][1], poly[j][0], poly[j][1],
                        self.goal_verts[k][0], self.goal_verts[k][1],
                        self.goal_verts[l_idx][0], self.goal_verts[l_idx][1]
                    ):
                        return True
            return False

    def _get_all_obstacles_for_layer(self, layer, obs):
        """Get all obstacles (static + permanent + active NOTAMs) for a layer."""
        all_obs = list(self.obstacles)

        for c in obs.active_constraints:
            try:
                cid = c.id if hasattr(c, 'id') else str(id(c))
                phase = str(c.phase)

                if 'RESTRICTED' not in phase and 'CONTROLLED' not in phase:
                    continue

                alts = list(c.alt_layers) if hasattr(c, 'alt_layers') and c.alt_layers else [1, 2, 3, 4]

                region = c.region
                region_data = None
                kind = None
                try:
                    region_data = (region.center_pos.x, region.center_pos.y, region.radius)
                    kind = 'circle'
                except Exception:
                    try:
                        region_data = [(v.x, v.y) for v in region.vertices]
                        kind = 'polygon'
                    except Exception:
                        continue

                if (layer == self.target_alt and layer in alts and
                        self._constraint_overlaps_goal(region_data, kind)):
                    goal_inside = False
                    if kind == 'circle':
                        cx_c, cy_c, r_c = region_data
                        goal_inside = point_in_circle(
                            self.goal[0], self.goal[1], cx_c, cy_c, r_c)
                    else:
                        goal_inside = point_in_polygon(
                            self.goal[0], self.goal[1], region_data)
                    if goal_inside:
                        continue

                ob = Obstacle(kind, region_data, alt_layers=alts, phase=phase)
                all_obs.append(ob)
                self.known_notams[cid] = ob

            except Exception:
                pass

        return all_obs

    # =========================================================================
    # LAYER SELECTION v1 (detour-proportional scoring from original best policy)
    # =========================================================================

    def _pick_best_layer(self, pos, obs):
        """Pick the altitude layer with shortest estimated path to goal.
        Uses detour-proportional penalties and intermediate goals for
        non-target layers (since the drone will switch near the goal)."""
        px, py = pos
        goal_dist = dist(px, py, self.goal[0], self.goal[1])

        # Determine optimal switch distance based on nearby NOTAMs (incl ADVISORY)
        switch_dist = 2500  # Default
        for c in obs.active_constraints:
            try:
                phase = str(c.phase)
                if 'ADVISORY' not in phase and 'CONTROLLED' not in phase and 'RESTRICTED' not in phase:
                    continue
                alts = list(c.alt_layers) if hasattr(c, 'alt_layers') and c.alt_layers else []
                region = c.region
                try:
                    cx_c, cy_c = region.center_pos.x, region.center_pos.y
                    r_c = region.radius
                except Exception:
                    try:
                        verts = [(v.x, v.y) for v in region.vertices]
                        cx_c = sum(v[0] for v in verts) / len(verts)
                        cy_c = sum(v[1] for v in verts) / len(verts)
                        r_c = max(dist(cx_c, cy_c, v[0], v[1]) for v in verts)
                    except Exception:
                        continue
                notam_to_goal = dist(cx_c, cy_c, self.goal[0], self.goal[1])
                if notam_to_goal < r_c + 5000:
                    if self.target_alt in alts:
                        switch_dist = min(switch_dist, 500)
                    else:
                        switch_dist = max(switch_dist, r_c + 1000)
            except Exception:
                pass

        if goal_dist < switch_dist:
            return self.target_alt

        best_layer = self.current_layer
        best_score = float('inf')

        for layer in [1, 2, 3, 4]:
            # For non-target layers, drone only needs to get within ~5000m
            # of goal (then switch to target_alt for final approach)
            if layer != self.target_alt and goal_dist > 5000:
                dx = self.goal[0] - px
                dy = self.goal[1] - py
                d = math.hypot(dx, dy)
                ratio = max(0, (d - 5000)) / d
                check_x = px + dx * ratio
                check_y = py + dy * ratio
            else:
                check_x, check_y = self.goal

            # Base score = straight-line distance to check point
            score = dist(px, py, check_x, check_y)

            all_obs = self._get_all_obstacles_for_layer(layer, obs)
            for ob in all_obs:
                if not ob.blocks_layer(layer):
                    continue

                sd = ob.signed_dist(px, py)

                # Currently inside an obstacle on this layer = catastrophic
                if sd < 0:
                    score += 50000
                    continue

                # Check if obstacle blocks the direct path to check point
                if ob.seg_intersects(px, py, check_x, check_y, OBSTACLE_BUFFER * 0.3):
                    # Estimate detour proportional to obstacle size
                    if ob.kind == 'circle':
                        _, _, r = ob.data
                        detour_cost = r * 2.5
                    else:
                        verts = ob.data
                        perim = 0
                        for i in range(len(verts)):
                            j = (i + 1) % len(verts)
                            perim += dist(verts[i][0], verts[i][1],
                                          verts[j][0], verts[j][1])
                        detour_cost = perim * 0.6
                    score += detour_cost
                elif sd < 500:
                    # Proximity penalty for nearby obstacles
                    score += (500 - sd) * 0.3

            # Traffic on this layer
            for t in obs.traffic_tracks:
                try:
                    if abs(layer - t.alt_layer) <= 1:
                        td = dist(px, py, t.position.x, t.position.y)
                        if td < TRAFFIC_DANGER_DIST * 3:
                            score += (TRAFFIC_DANGER_DIST * 3 - td) * 2
                except Exception:
                    pass

            # Soft penalty for ADVISORY NOTAMs on this layer (predictive avoidance)
            for c in obs.active_constraints:
                try:
                    phase = str(c.phase)
                    if 'ADVISORY' not in phase:
                        continue
                    alts = list(c.alt_layers) if hasattr(c, 'alt_layers') and c.alt_layers else []
                    if layer not in alts:
                        continue
                    region = c.region
                    try:
                        cx_c, cy_c = region.center_pos.x, region.center_pos.y
                        r_c = region.radius
                    except Exception:
                        try:
                            verts_c = [(v.x, v.y) for v in region.vertices]
                            cx_c = sum(v[0] for v in verts_c) / len(verts_c)
                            cy_c = sum(v[1] for v in verts_c) / len(verts_c)
                            r_c = max(dist(cx_c, cy_c, v[0], v[1]) for v in verts_c)
                        except Exception:
                            continue
                    # If ADVISORY NOTAM is between drone and goal, add soft penalty
                    notam_dist = dist(px, py, cx_c, cy_c)
                    if notam_dist < r_c + 3000:
                        # Soft penalty: enough to prefer other layers but not block
                        score += (r_c + 3000 - notam_dist) * 0.5
                except Exception:
                    pass

            # Moderate hysteresis to prevent oscillation
            if layer != self.current_layer:
                score += 500

            # Prefer target_alt when getting close to goal
            if goal_dist < 8000 and layer == self.target_alt:
                score -= 2000

            if score < best_score:
                best_score = score
                best_layer = layer

        return best_layer

    def _replan(self, pos, obs):
        """Recompute visibility graph path from current position to goal."""
        all_obs = self._get_all_obstacles_for_layer(self.current_layer, obs)

        self.path = build_visibility_graph(
            pos, self.goal, all_obs, self.current_layer, self.map_bounds,
        )
        self.path_idx = 1 if len(self.path) > 1 else 0
        self.last_replan_tick = self.tick

    def _should_replan(self, obs):
        """Check if we need to replan. Reacts to:
        1. Timer (every REPLAN_INTERVAL ticks)
        2. ANY new constraint appearing (including ADVISORY)
        3. ANY constraint phase change (escalation or de-escalation)
        4. ANY constraint disappearing
        5. New NPC traffic appearing on or near our planned path
        """
        if self.tick - self.last_replan_tick >= REPLAN_INTERVAL:
            return True

        # --- Track ALL constraints by ID and phase ---
        current_constraint_ids = set()
        phase_changed = False
        current_phases = {}
        for c in obs.active_constraints:
            try:
                cid = c.id if hasattr(c, 'id') else str(id(c))
                phase = str(c.phase)
                current_constraint_ids.add(cid)
                current_phases[cid] = phase

                # Track phase history
                if cid not in self.notam_history:
                    self.notam_history[cid] = {}
                hist = self.notam_history[cid]
                if 'ADVISORY' in phase and 'advisory_tick' not in hist:
                    hist['advisory_tick'] = self.tick
                if 'CONTROLLED' in phase and 'controlled_tick' not in hist:
                    hist['controlled_tick'] = self.tick
                if 'RESTRICTED' in phase and 'restricted_tick' not in hist:
                    hist['restricted_tick'] = self.tick

                # Detect phase change
                if cid in self.last_notam_phases and self.last_notam_phases[cid] != phase:
                    phase_changed = True
            except Exception:
                pass

        # New constraint appeared or old one disappeared
        new_constraints = current_constraint_ids - self.last_constraint_ids
        lost_constraints = self.last_constraint_ids - current_constraint_ids
        self.last_constraint_ids = current_constraint_ids
        self.last_notam_phases = current_phases

        if new_constraints or lost_constraints or phase_changed:
            return True

        # --- Track NPC traffic: replan if new NPC near our path ---
        current_traffic_ids = set()
        px = obs.ownship_state.position.x
        py = obs.ownship_state.position.y
        for t in obs.traffic_tracks:
            try:
                tid = t.id if hasattr(t, 'id') else str(id(t))
                current_traffic_ids.add(tid)

                # If this is a NEW NPC, check if it's near our planned path
                if tid not in self.last_traffic_ids and len(self.path) > 1:
                    tx, ty = t.position.x, t.position.y
                    t_alt = t.alt_layer
                    if abs(t_alt - self.current_layer) <= 1:
                        # Check if NPC is near any segment of our remaining path
                        for i in range(max(0, self.path_idx - 1), len(self.path) - 1):
                            ax, ay = self.path[i]
                            bx, by = self.path[i + 1]
                            if seg_intersects_circle(ax, ay, bx, by,
                                                     tx, ty, TRAFFIC_DANGER_DIST):
                                self.last_traffic_ids = current_traffic_ids
                                return True
            except Exception:
                pass
        self.last_traffic_ids = current_traffic_ids

        # --- Check if NPC predicted trajectory intersects our planned path ---
        if len(self.path) > 1 and self.path_idx < len(self.path):
            next_wp = self.path[self.path_idx]
            for t in obs.traffic_tracks:
                try:
                    t_alt = t.alt_layer
                    if abs(t_alt - self.current_layer) > 1:
                        continue
                    tx, ty = t.position.x, t.position.y
                    # Check if NPC is heading toward our next waypoint
                    if hasattr(t, 'velocity') and t.velocity:
                        vx, vy = t.velocity.x, t.velocity.y
                        speed_sq = vx * vx + vy * vy
                        if speed_sq > 1:
                            # Predict NPC position 20 ticks ahead
                            fut_x = tx + vx * 20
                            fut_y = ty + vy * 20
                            # If NPC trajectory passes near our next waypoint
                            if seg_intersects_circle(tx, ty, fut_x, fut_y,
                                                     next_wp[0], next_wp[1],
                                                     CONFLICT_DIST):
                                return True
                except Exception:
                    pass

        return False

    def _is_in_restricted(self, px, py, alt, obs):
        """Check if position is inside a RESTRICTED zone on matching layer."""
        for c in obs.active_constraints:
            try:
                phase = str(c.phase)
                if 'RESTRICTED' not in phase:
                    continue
                alts = list(c.alt_layers) if hasattr(c, 'alt_layers') and c.alt_layers else []
                if alt not in alts:
                    continue
                region = c.region
                try:
                    cx, cy, r = region.center_pos.x, region.center_pos.y, region.radius
                    if point_in_circle(px, py, cx, cy, r):
                        return True
                except Exception:
                    try:
                        verts = [(v.x, v.y) for v in region.vertices]
                        if point_in_polygon(px, py, verts):
                            return True
                    except Exception:
                        pass
            except Exception:
                pass
        return False

    def _is_in_constraint(self, px, py, alt, obs):
        """Check if position is inside any CONTROLLED or RESTRICTED zone."""
        for c in obs.active_constraints:
            try:
                phase = str(c.phase)
                if 'RESTRICTED' not in phase and 'CONTROLLED' not in phase:
                    continue
                alts = list(c.alt_layers) if hasattr(c, 'alt_layers') and c.alt_layers else []
                if alt not in alts:
                    continue
                region = c.region
                try:
                    cx, cy, r = region.center_pos.x, region.center_pos.y, region.radius
                    if point_in_circle(px, py, cx, cy, r):
                        return True
                except Exception:
                    try:
                        verts = [(v.x, v.y) for v in region.vertices]
                        if point_in_polygon(px, py, verts):
                            return True
                    except Exception:
                        pass
            except Exception:
                pass

        for ob in self.obstacles:
            if ob.blocks_layer(alt) and ob.contains(px, py):
                return True

        return False

    def _emergency_escape_direction(self, px, py, alt, obs):
        """Compute direction to escape restricted zone ASAP."""
        best_dx, best_dy = 0, 0
        best_score = float('inf')

        for i in range(16):
            angle = 2 * math.pi * i / 16
            test_x = px + math.cos(angle) * SPEED * 5
            test_y = py + math.sin(angle) * SPEED * 5

            min_x, max_x, min_y, max_y = self.map_bounds
            test_x = max(min_x + MAP_MARGIN, min(max_x - MAP_MARGIN, test_x))
            test_y = max(min_y + MAP_MARGIN, min(max_y - MAP_MARGIN, test_y))

            score = 0
            for c in obs.active_constraints:
                try:
                    phase = str(c.phase)
                    if 'RESTRICTED' not in phase:
                        continue
                    alts = list(c.alt_layers) if hasattr(c, 'alt_layers') and c.alt_layers else []
                    if alt not in alts:
                        continue
                    region = c.region
                    try:
                        cx, cy, r = region.center_pos.x, region.center_pos.y, region.radius
                        d_from_center = dist(test_x, test_y, cx, cy)
                        if d_from_center < r:
                            score += (r - d_from_center) * 10
                        else:
                            score -= (d_from_center - r) * 5
                    except Exception:
                        try:
                            verts = [(v.x, v.y) for v in region.vertices]
                            ob = Obstacle('polygon', verts)
                            sd = ob.signed_dist(test_x, test_y)
                            if sd < 0:
                                score += abs(sd) * 10
                            else:
                                score -= sd * 5
                        except Exception:
                            pass
                except Exception:
                    pass

            score += dist(test_x, test_y, self.goal[0], self.goal[1]) * 0.01

            if score < best_score:
                best_score = score
                best_dx = math.cos(angle)
                best_dy = math.sin(angle)

        return best_dx * SPEED, best_dy * SPEED

    def _find_safe_alt(self, px, py, desired_alt, obs, cur_alt=None):
        """If desired layer is blocked, find an alternative.
        When ALL layers are blocked, prefer: current physical alt > layer with
        lowest penalty severity > desired_alt."""
        if not self._is_in_constraint(px, py, desired_alt, obs):
            return desired_alt
        # Try to find a completely safe layer
        for alt in [1, 2, 3, 4]:
            if alt != desired_alt and not self._is_in_constraint(px, py, alt, obs):
                return alt
        # ALL layers blocked — pick the least dangerous one
        # Prefer current physical alt (no transition needed), then layers
        # not in any RESTRICTED permanent constraint
        if cur_alt is not None:
            # Check if current alt is in a permanent RESTRICTED vs just CONTROLLED/ADVISORY
            cur_in_perm_restricted = False
            for ob in self.obstacles:
                if not ob.is_static and ob.blocks_layer(cur_alt) and ob.contains(px, py):
                    cur_in_perm_restricted = True
                    break
            if not cur_in_perm_restricted:
                return cur_alt
            # Current alt is in a permanent restricted zone — find layer NOT in permanent restricted
            for alt in [1, 2, 3, 4]:
                in_perm = False
                for ob in self.obstacles:
                    if not ob.is_static and ob.blocks_layer(alt) and ob.contains(px, py):
                        in_perm = True
                        break
                if not in_perm:
                    return alt
        # Still no good option — prefer current physical alt if available
        if cur_alt is not None:
            return cur_alt
        return desired_alt

    def _compute_move(self, px, py, tx, ty, avx, avy):
        """Compute final position after moving toward target with avoidance."""
        dx = tx - px
        dy = ty - py
        d_to_target = math.hypot(dx, dy)

        if d_to_target < 1e-6:
            return px, py

        ux, uy = dx / d_to_target, dy / d_to_target

        bx = ux * SPEED + avx * 0.5
        by = uy * SPEED + avy * 0.5
        bm = math.hypot(bx, by)

        if bm > 1e-6:
            step_d = min(SPEED, d_to_target)
            mx = bx / bm * step_d
            my = by / bm * step_d
        else:
            mx = ux * min(SPEED, d_to_target)
            my = uy * min(SPEED, d_to_target)

        min_x, max_x, min_y, max_y = self.map_bounds
        nx = max(min_x + MAP_MARGIN, min(max_x - MAP_MARGIN, px + mx))
        ny = max(min_y + MAP_MARGIN, min(max_y - MAP_MARGIN, py + my))
        return nx, ny

    def _safe_position(self, from_x, from_y, to_x, to_y, alt, obs):
        """Ensure the target position doesn't enter any obstacle or restricted constraint.
        EXCEPTION: Allow entry into goal region (accept minor penalties to reach goal)."""
        if self._point_in_goal(to_x, to_y):
            for ob in self.obstacles:
                if ob.is_static and ob.contains(to_x, to_y):
                    for frac in [0.8, 0.6, 0.4, 0.2, 0.0]:
                        test_x = from_x + frac * (to_x - from_x)
                        test_y = from_y + frac * (to_y - from_y)
                        if not ob.contains(test_x, test_y):
                            return test_x, test_y
                    return from_x, from_y
            return to_x, to_y

        for ob in self.obstacles:
            if not ob.blocks_layer(alt):
                continue
            inside_end = ob.contains(to_x, to_y)
            crosses_path = ob.seg_intersects(from_x, from_y, to_x, to_y, 0)
            if inside_end or crosses_path:
                for frac in [0.8, 0.6, 0.4, 0.2, 0.0]:
                    test_x = from_x + frac * (to_x - from_x)
                    test_y = from_y + frac * (to_y - from_y)
                    if not ob.contains(test_x, test_y):
                        cx, cy = ob.centroid()
                        dd = dist(test_x, test_y, cx, cy)
                        if dd > 1e-6:
                            push = OBSTACLE_BUFFER * 0.3
                            test_x += (test_x - cx) / dd * push
                            test_y += (test_y - cy) / dd * push
                        return test_x, test_y
                return from_x, from_y

        goal_dist_here = dist(from_x, from_y, self.goal[0], self.goal[1])
        for c in obs.active_constraints:
            try:
                phase = str(c.phase)
                if 'RESTRICTED' not in phase:
                    continue
                alts = list(c.alt_layers) if hasattr(c, 'alt_layers') and c.alt_layers else []
                if alt not in alts:
                    continue

                region = c.region
                region_data = None
                kind = None
                inside = False
                try:
                    cx, cy, r = region.center_pos.x, region.center_pos.y, region.radius
                    inside = point_in_circle(to_x, to_y, cx, cy, r)
                    region_data = (cx, cy, r)
                    kind = 'circle'
                except Exception:
                    try:
                        verts = [(v.x, v.y) for v in region.vertices]
                        inside = point_in_polygon(to_x, to_y, verts)
                        region_data = verts
                        kind = 'polygon'
                    except Exception:
                        pass

                # Skip goal-overlapping constraints on final approach at target alt
                if (inside and alt == self.target_alt and goal_dist_here < 5000 and
                        region_data and kind and
                        self._constraint_overlaps_goal(region_data, kind)):
                    continue

                if inside:
                    for frac in [0.8, 0.6, 0.4, 0.2, 0.0]:
                        test_x = from_x + frac * (to_x - from_x)
                        test_y = from_y + frac * (to_y - from_y)
                        try:
                            if not point_in_circle(test_x, test_y, cx, cy, r):
                                return test_x, test_y
                        except Exception:
                            try:
                                if not point_in_polygon(test_x, test_y, verts):
                                    return test_x, test_y
                            except Exception:
                                pass
                    return from_x, from_y
            except Exception:
                pass

        return to_x, to_y

    def _nearest_emergency_site(self, px, py):
        """Find nearest emergency landing site."""
        best_d = float('inf')
        best = None
        for sx, sy in self.emergency_sites:
            dd = dist(px, py, sx, sy)
            if dd < best_d:
                best_d = dd
                best = (sx, sy)
        return best, best_d

    def step(self, obs: Observation) -> Plan:
        self.tick += 1

        try:
            return self._step_inner(obs)
        except Exception:
            import traceback, sys
            traceback.print_exc(file=sys.stderr)
            return Plan(steps=[ActionStep(action_type=ActionType.HOLD)] * 5)

    def _step_inner(self, obs):
        if not self.initialized:
            try:
                self._init(obs)
            except Exception as e:
                import traceback
                traceback.print_exc()
                self.initialized = True
                if self.goal is None:
                    self.goal = (37000.0, 36000.0)
                if not self.goal_verts:
                    self.goal_verts = []
                if not self.path:
                    px = obs.ownship_state.position.x
                    py = obs.ownship_state.position.y
                    self.path = [(px, py), self.goal]
                    self.path_idx = 1

        px = obs.ownship_state.position.x
        py = obs.ownship_state.position.y
        cur_alt = obs.ownship_state.alt_layer
        energy = obs.ownship_state.energy

        # Dynamically adjust goal point to avoid RESTRICTED NOTAMs
        if self.tick % 100 == 50:
            self._adjust_goal_for_notams(obs)

        goal_dist = dist(px, py, self.goal[0], self.goal[1])

        # =====================================================================
        # ENERGY CHECK -- bail to emergency landing if we can't make it
        # =====================================================================
        ticks_of_fuel = energy / max(self.energy_decay, 0.001)
        ticks_to_goal = goal_dist / SPEED if SPEED > 0 else 99999

        if ticks_of_fuel < ticks_to_goal * 1.15:
            site, site_dist = self._nearest_emergency_site(px, py)
            ticks_to_site = site_dist / SPEED if site and SPEED > 0 else 99999

            if ticks_of_fuel < ticks_to_goal * 1.05 and site and ticks_to_site < ticks_of_fuel * 0.8:
                if dist(px, py, site[0], site[1]) < SPEED * 2:
                    step0 = ActionStep(action_type=ActionType.EMERGENCY_LAND)
                    return Plan(steps=[step0, step0,
                                      ActionStep(action_type=ActionType.HOLD),
                                      ActionStep(action_type=ActionType.HOLD),
                                      ActionStep(action_type=ActionType.HOLD)])
                nx, ny = self._compute_move(px, py, site[0], site[1], 0, 0)
                step0 = ActionStep(
                    action_type=ActionType.WAYPOINT,
                    target_position=Position2D(x=nx, y=ny),
                    target_alt_layer=cur_alt,
                )
                return Plan(steps=[step0, step0,
                                   ActionStep(action_type=ActionType.HOLD),
                                   ActionStep(action_type=ActionType.HOLD),
                                   ActionStep(action_type=ActionType.HOLD)])

        # =====================================================================
        # RESTRICTED ZONE TRACKING -- escape before hitting 5 consecutive
        # =====================================================================
        in_restricted = self._is_in_restricted(px, py, cur_alt, obs)
        if in_restricted:
            self.restricted_consecutive += 1
        else:
            self.restricted_consecutive = 0

        if self.restricted_consecutive >= RESTRICTED_ESCAPE_THRESHOLD:
            for alt in [1, 2, 3, 4]:
                if alt != cur_alt and not self._is_in_restricted(px, py, alt, obs):
                    escape_step = ActionStep(
                        action_type=ActionType.WAYPOINT,
                        target_position=Position2D(x=px, y=py),
                        target_alt_layer=alt,
                    )
                    return Plan(steps=[escape_step, escape_step,
                                       ActionStep(action_type=ActionType.HOLD),
                                       ActionStep(action_type=ActionType.HOLD),
                                       ActionStep(action_type=ActionType.HOLD)])

            esc_dx, esc_dy = self._emergency_escape_direction(px, py, cur_alt, obs)
            nx = px + esc_dx
            ny = py + esc_dy
            min_x, max_x, min_y, max_y = self.map_bounds
            nx = max(min_x + MAP_MARGIN, min(max_x - MAP_MARGIN, nx))
            ny = max(min_y + MAP_MARGIN, min(max_y - MAP_MARGIN, ny))
            escape_step = ActionStep(
                action_type=ActionType.WAYPOINT,
                target_position=Position2D(x=nx, y=ny),
                target_alt_layer=cur_alt,
            )
            return Plan(steps=[escape_step, escape_step,
                               ActionStep(action_type=ActionType.HOLD),
                               ActionStep(action_type=ActionType.HOLD),
                               ActionStep(action_type=ActionType.HOLD)])

        # =====================================================================
        # LAYER + PATH REPLANNING
        # =====================================================================
        needs_replan = self._should_replan(obs)
        if self.tick % REPLAN_INTERVAL == 0 or self.tick <= 2 or needs_replan:
            new_layer = self._pick_best_layer((px, py), obs)
            if new_layer != self.current_layer:
                # Safety: don't switch to a layer where we're inside a permanent constraint
                in_perm_constraint = False
                for ob in self.obstacles:
                    if not ob.is_static and ob.blocks_layer(new_layer) and ob.contains(px, py):
                        in_perm_constraint = True
                        break
                if not in_perm_constraint:
                    self.current_layer = new_layer
                    needs_replan = True
                # else: stay on current_layer, don't replan for a bad layer

        if needs_replan:
            self._replan((px, py), obs)

        # =====================================================================
        # FOLLOW VISIBILITY GRAPH PATH
        # =====================================================================
        while self.path_idx < len(self.path) - 1:
            wp = self.path[self.path_idx]
            if dist(px, py, wp[0], wp[1]) < WP_REACH_DIST:
                self.path_idx += 1
            else:
                break

        if self.path_idx < len(self.path):
            tx, ty = self.path[self.path_idx]
        else:
            tx, ty = self.goal

        # =====================================================================
        # WAYPOINT REACHABILITY CHECK + SKIP
        # =====================================================================
        fuel_ratio = ticks_of_fuel / max(ticks_to_goal, 1)
        if len(self.path) > 1:
            all_obs_for_skip = self._get_all_obstacles_for_layer(self.current_layer, obs)

            if self.path_idx < len(self.path):
                cur_wp = self.path[self.path_idx]
                cur_blocked = False
                for ob in all_obs_for_skip:
                    if ob.blocks_layer(self.current_layer) and ob.is_static:
                        if ob.seg_intersects(px, py, cur_wp[0], cur_wp[1], OBSTACLE_BUFFER * 0.3):
                            cur_blocked = True
                            break
                if cur_blocked:
                    self._replan((px, py), obs)
                    if self.path_idx < len(self.path):
                        tx, ty = self.path[self.path_idx]
                    else:
                        tx, ty = self.goal

            best_skip = self.path_idx
            for skip_idx in range(len(self.path) - 1, self.path_idx, -1):
                wp = self.path[skip_idx]
                clear = True
                for ob in all_obs_for_skip:
                    if ob.blocks_layer(self.current_layer):
                        if ob.seg_intersects(px, py, wp[0], wp[1], OBSTACLE_BUFFER * 0.3):
                            clear = False
                            break
                if clear:
                    best_skip = skip_idx
                    break
            if best_skip > self.path_idx:
                self.path_idx = best_skip

        # =====================================================================
        # ALTITUDE
        # =====================================================================
        alt_switch_dist = 2500
        for c in obs.active_constraints:
            try:
                phase = str(c.phase)
                if ('ADVISORY' not in phase and 'CONTROLLED' not in phase
                        and 'RESTRICTED' not in phase):
                    continue
                alts = list(c.alt_layers) if hasattr(c, 'alt_layers') and c.alt_layers else []
                region = c.region
                try:
                    cx_c, cy_c = region.center_pos.x, region.center_pos.y
                    r_c = region.radius
                except Exception:
                    try:
                        verts = [(v.x, v.y) for v in region.vertices]
                        cx_c = sum(v[0] for v in verts) / len(verts)
                        cy_c = sum(v[1] for v in verts) / len(verts)
                        r_c = max(dist(cx_c, cy_c, v[0], v[1]) for v in verts)
                    except Exception:
                        continue
                notam_to_goal = dist(cx_c, cy_c, self.goal[0], self.goal[1])
                if notam_to_goal < r_c + 5000:
                    if self.target_alt in alts:
                        alt_switch_dist = min(alt_switch_dist, 500)
                    else:
                        alt_switch_dist = max(alt_switch_dist, r_c + 1000)
            except Exception:
                pass

        if goal_dist < min(alt_switch_dist, 500):
            desired_alt = self.target_alt
            desired_alt = self._find_safe_alt(px, py, desired_alt, obs, cur_alt)
        elif goal_dist < alt_switch_dist:
            desired_alt = self.target_alt
            desired_alt = self._find_safe_alt(px, py, desired_alt, obs, cur_alt)
        else:
            desired_alt = self.current_layer
            desired_alt = self._find_safe_alt(px, py, desired_alt, obs, cur_alt)

        # =====================================================================
        # FINAL APPROACH MODE -- reduce avoidance near goal for speed
        # =====================================================================
        near_goal = goal_dist < 2000
        approach_damping = 0.2 if near_goal else 1.0

        # =====================================================================
        # CONE TRAFFIC AVOIDANCE (replaces CPA)
        # =====================================================================
        d_to_target = dist(px, py, tx, ty)
        if d_to_target > 1e-6:
            pvx = (tx - px) / d_to_target * SPEED
            pvy = (ty - py) / d_to_target * SPEED
        else:
            pvx, pvy = 0, 0

        avx, avy = cone_avoidance_vector(px, py, pvx, pvy, obs.traffic_tracks, desired_alt)
        avx *= approach_damping
        avy *= approach_damping

        # =====================================================================
        # CONSTRAINT PUSH -- push away from nearby NOTAMs/constraints
        # =====================================================================
        for c in obs.active_constraints:
            try:
                phase = str(c.phase)
                if 'ADVISORY' not in phase and 'CONTROLLED' not in phase and 'RESTRICTED' not in phase:
                    continue

                alts = list(c.alt_layers) if hasattr(c, 'alt_layers') and c.alt_layers else []
                if desired_alt not in alts:
                    continue

                if 'RESTRICTED' in phase:
                    w = 3.0
                elif 'CONTROLLED' in phase:
                    w = 1.5
                else:
                    w = 0.3

                region = c.region
                try:
                    cx, cy = region.center_pos.x, region.center_pos.y
                    r = region.radius
                    sd = math.hypot(px - cx, py - cy) - r
                    region_data = (cx, cy, r)
                    kind = 'circle'
                except Exception:
                    try:
                        verts = [(v.x, v.y) for v in region.vertices]
                        ob = Obstacle('polygon', verts)
                        sd = ob.signed_dist(px, py)
                        cx, cy = ob.centroid()
                        region_data = verts
                        kind = 'polygon'
                    except Exception:
                        continue

                # Skip push for goal-overlapping constraints on final approach
                if (desired_alt == self.target_alt and goal_dist < 5000 and
                        self._constraint_overlaps_goal(region_data, kind)):
                    continue

                push_radius = CONSTRAINT_BUFFER * 1.5
                if sd < push_radius:
                    dd = dist(px, py, cx, cy)
                    if dd > 1e-6:
                        strength = w * max(0, push_radius - sd) * 0.3 * approach_damping
                        avx += (px - cx) / dd * strength
                        avy += (py - cy) / dd * strength

            except Exception:
                pass

        # =====================================================================
        # STATIC OBSTACLE AVOIDANCE -- reactive push away from static obstacles
        # =====================================================================
        for ob in self.obstacles:
            if ob.is_static and ob.blocks_layer(desired_alt):
                sd = ob.signed_dist(px, py)
                push_dist = OBSTACLE_BUFFER * 3.0
                if sd < push_dist:
                    cx, cy = ob.centroid()
                    dd = dist(px, py, cx, cy)
                    if dd > 1e-6:
                        if sd < 0:
                            strength = SPEED * 2.0
                        else:
                            strength = max(0, push_dist - sd) * 0.5
                        avx += (px - cx) / dd * strength
                        avy += (py - cy) / dd * strength

        # =====================================================================
        # COMPUTE FINAL MOVE (with safety check)
        # =====================================================================
        nx, ny = self._compute_move(px, py, tx, ty, avx, avy)
        nx, ny = self._safe_position(px, py, nx, ny, desired_alt, obs)

        # Lookahead for step[1]
        d_to_next = dist(nx, ny, tx, ty)
        if d_to_next < WP_REACH_DIST and self.path_idx + 1 < len(self.path):
            tx2, ty2 = self.path[self.path_idx + 1]
        else:
            tx2, ty2 = tx, ty

        d_to_t2 = dist(nx, ny, tx2, ty2)
        if d_to_t2 > 1e-6:
            pvx2 = (tx2 - nx) / d_to_t2 * SPEED
            pvy2 = (ty2 - ny) / d_to_t2 * SPEED
        else:
            pvx2, pvy2 = 0, 0

        avx2, avy2 = cone_avoidance_vector(nx, ny, pvx2, pvy2, obs.traffic_tracks, desired_alt)
        nx2, ny2 = self._compute_move(nx, ny, tx2, ty2, avx2, avy2)
        nx2, ny2 = self._safe_position(nx, ny, nx2, ny2, desired_alt, obs)

        # =====================================================================
        # BUILD PLAN
        # =====================================================================
        step0 = ActionStep(
            action_type=ActionType.WAYPOINT,
            target_position=Position2D(x=nx, y=ny),
            target_alt_layer=desired_alt,
        )
        step1 = ActionStep(
            action_type=ActionType.WAYPOINT,
            target_position=Position2D(x=nx2, y=ny2),
            target_alt_layer=desired_alt,
        )
        hold = ActionStep(action_type=ActionType.HOLD)

        return Plan(steps=[step0, step1, hold, hold, hold])
