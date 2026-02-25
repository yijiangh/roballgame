import math
import random
try:
    # Use tkinter for a simple desktop visualization.
    import tkinter as tk
except ModuleNotFoundError:
    # Allow import in environments without Tk; startup will fail with a clear error.
    tk = None

# Simulation timing and playfield size.
WIDTH = 900
HEIGHT = 600
FPS = 60
DT = 1.0 / FPS

# Agent movement limits.
DOT_R = 12
MAX_SPEED = 240.0
ACCEL = 900.0

# Distance thresholds for slowdown and stop behavior.
SLOW_DIST = 30.0
STOP_DIST = 1.0

# Repulsive-field tuning values.
REPEL_DIST = 160.0
REPEL_K = 6000.0
REPEL_MAX = 600.0

# UI colors.
BG = "#0f1218"
FG = "#d7dde5"
ACCENT = "#7cd4ff"
WARN = "#ff7c7c"

# Labels shown in the mode selector and HUD.
MODE_NAMES = {
    1: "Speed Scaling",
    2: "Repulsive Field",
    3: "Project Normal",
    4: "Damped Barrier",
}

PARAM_STEP = 5.0
REPEL_STEP = 500.0
CSV_PATH = "logs/vel_dist.csv"
CLEARANCE = 2.0


def clamp(x, a, b):
    """Return x clamped to [a, b]. Inputs: scalar x, lower bound a, upper bound b."""
    # Clamp x to the inclusive range [a, b].
    return max(a, min(b, x))


def vec_len(v):
    """Return length of vector v. Input: v as (x, y)."""
    # 2D vector magnitude.
    return math.hypot(v[0], v[1])


def vec_add(a, b):
    """Return a + b. Inputs: vectors a and b as (x, y)."""
    return (a[0] + b[0], a[1] + b[1])


def vec_sub(a, b):
    """Return a - b. Inputs: vectors a and b as (x, y)."""
    return (a[0] - b[0], a[1] - b[1])


def vec_mul(a, s):
    """Return vector a scaled by s. Inputs: vector a, scalar s."""
    return (a[0] * s, a[1] * s)


def vec_dot(a, b):
    """Return dot product of vectors a and b. Inputs: 2D vectors."""
    return a[0] * b[0] + a[1] * b[1]


def vec_norm(a):
    """Return normalized vector of a. Input: vector a as (x, y)."""
    # Return a unit vector (or zero vector for near-zero input).
    n = vec_len(a)
    if n < 1e-8:
        return (0.0, 0.0)
    return (a[0] / n, a[1] / n)


def distance_to_obstacle(p, obs, extra_r):
    """Distance from point p to obstacle obs expanded by extra_r.

    Inputs:
    p: point as (x, y)
    obs: obstacle instance (circle/wall/rect/segment)
    extra_r: radius expansion used for clearance checks
    """
    # Generic point-to-obstacle distance used for placement checks.
    px, py = p
    if isinstance(obs, CircleObstacle):
        dx = px - obs.x
        dy = py - obs.y
        return math.hypot(dx, dy) - (obs.r + extra_r)
    if isinstance(obs, WallObstacle):
        if obs.side == "left":
            return px - extra_r
        if obs.side == "right":
            return (WIDTH - px) - extra_r
        if obs.side == "top":
            return py - extra_r
        if obs.side == "bottom":
            return (HEIGHT - py) - extra_r
    if isinstance(obs, RectObstacle):
        left = obs.x - extra_r
        right = obs.x + obs.w + extra_r
        top = obs.y - extra_r
        bottom = obs.y + obs.h + extra_r
        cx = clamp(px, left, right)
        cy = clamp(py, top, bottom)
        return math.hypot(px - cx, py - cy)
    if isinstance(obs, SegmentObstacle):
        ax, ay = obs.ax, obs.ay
        bx, by = obs.bx, obs.by
        abx = bx - ax
        aby = by - ay
        apx = px - ax
        apy = py - ay
        ab_len2 = abx * abx + aby * aby
        if ab_len2 < 1e-8:
            return math.hypot(px - ax, py - ay) - extra_r
        t = clamp((apx * abx + apy * aby) / ab_len2, 0.0, 1.0)
        cx = ax + t * abx
        cy = ay + t * aby
        return math.hypot(px - cx, py - cy) - extra_r
    return 1e9


class CircleObstacle:
    def __init__(self, x, y, r):
        """Create a circle obstacle. Inputs: center (x, y), radius r."""
        self.x = x
        self.y = y
        self.r = r

    def dist_and_normal(self, p):
        """Signed distance and normal for point p, where p is (x, y)."""
        # Signed distance from dot surface to circle surface, plus outward normal.
        dx = p[0] - self.x
        dy = p[1] - self.y
        d = math.hypot(dx, dy)
        if d < 1e-6:
            return -self.r, (1.0, 0.0)
        n = (dx / d, dy / d)
        return d - (self.r + DOT_R), n


class WallObstacle:
    # axis-aligned wall: normal points inward from boundary
    def __init__(self, side):
        """Create a boundary wall. Input: side in {'left','right','top','bottom'}."""
        self.side = side

    def dist_and_normal(self, p):
        """Signed distance and inward normal for point p, where p is (x, y)."""
        if self.side == "left":
            return p[0] - DOT_R, (1.0, 0.0)
        if self.side == "right":
            return (WIDTH - p[0]) - DOT_R, (-1.0, 0.0)
        if self.side == "top":
            return p[1] - DOT_R, (0.0, 1.0)
        if self.side == "bottom":
            return (HEIGHT - p[1]) - DOT_R, (0.0, -1.0)
        return 1e9, (0.0, 0.0)


class RectObstacle:
    # axis-aligned rectangle
    def __init__(self, x, y, w, h):
        """Create axis-aligned rectangle. Inputs: top-left (x, y), width w, height h."""
        self.x = x
        self.y = y
        self.w = w
        self.h = h

    def dist_and_normal(self, p):
        """Signed distance and normal for point p, where p is (x, y)."""
        # signed distance to rectangle expanded by DOT_R
        left = self.x - DOT_R
        right = self.x + self.w + DOT_R
        top = self.y - DOT_R
        bottom = self.y + self.h + DOT_R

        px, py = p
        inside_x = left <= px <= right
        inside_y = top <= py <= bottom

        if inside_x and inside_y:
            # inside expanded rect: negative distance, normal toward nearest side
            d_left = px - left
            d_right = right - px
            d_top = py - top
            d_bottom = bottom - py
            dmin = min(d_left, d_right, d_top, d_bottom)
            if dmin == d_left:
                return -d_left, (1.0, 0.0)
            if dmin == d_right:
                return -d_right, (-1.0, 0.0)
            if dmin == d_top:
                return -d_top, (0.0, 1.0)
            return -d_bottom, (0.0, -1.0)

        # outside: compute distance to nearest point on rect
        cx = clamp(px, left, right)
        cy = clamp(py, top, bottom)
        dx = px - cx
        dy = py - cy
        d = math.hypot(dx, dy)
        n = vec_norm((dx, dy))
        return d, n


class SegmentObstacle:
    def __init__(self, ax, ay, bx, by):
        """Create line segment from point A(ax, ay) to point B(bx, by)."""
        self.ax = ax
        self.ay = ay
        self.bx = bx
        self.by = by

    def dist_and_normal(self, p):
        """Signed distance and normal from point p=(x, y) to the segment."""
        # Distance to nearest point on the segment (expanded by DOT_R).
        ax, ay = self.ax, self.ay
        bx, by = self.bx, self.by
        px, py = p
        abx = bx - ax
        aby = by - ay
        apx = px - ax
        apy = py - ay
        ab_len2 = abx * abx + aby * aby
        if ab_len2 < 1e-8:
            dx = px - ax
            dy = py - ay
            d = math.hypot(dx, dy) - DOT_R
            return d, vec_norm((dx, dy))
        t = clamp((apx * abx + apy * aby) / ab_len2, 0.0, 1.0)
        cx = ax + t * abx
        cy = ay + t * aby
        dx = px - cx
        dy = py - cy
        d = math.hypot(dx, dy) - DOT_R
        n = vec_norm((dx, dy))
        return d, n


class Game:
    def __init__(self, root):
        """Initialize UI and simulation state. Input: Tk root window."""
        self.root = root
        # Top control bar + main drawing canvas.
        self.controls = tk.Frame(root, bg=BG)
        self.controls.pack(fill="x")
        self.canvas = tk.Canvas(root, width=WIDTH, height=HEIGHT, bg=BG, highlightthickness=0)
        self.canvas.pack()

        # Dot state: position, velocity, and commanded acceleration.
        self.dot = [WIDTH * 0.5, HEIGHT * 0.5]
        self.vel = [0.0, 0.0]
        self.cmd = [0.0, 0.0]

        # Runtime model parameters (user-adjustable).
        self.mode = 1
        self.slow_dist = SLOW_DIST
        self.stop_dist = STOP_DIST
        self.repel_k = REPEL_K
        self.obstacles = []
        self._init_scene()

        self.keys = set()
        # CSV log for post-run analysis.
        self.log_file = open(CSV_PATH, "w", encoding="utf-8")
        self.log_file.write("t,mode,dist,speed,cmd_speed,x,y\n")
        self.t = 0.0
        self._init_controls()
        root.protocol("WM_DELETE_WINDOW", self.on_close)
        root.bind("<KeyPress>", self.on_key_down)
        root.bind("<KeyRelease>", self.on_key_up)

        self.tick()

    def _init_controls(self):
        """Build control widgets. No external inputs."""
        label_fg = FG
        self.controls.configure(padx=10, pady=6)

        # Mode dropdown.
        self.mode_var = tk.StringVar(value=MODE_NAMES[self.mode])
        mode_label = tk.Label(self.controls, text="Control Model:", bg=BG, fg=label_fg)
        mode_label.grid(row=0, column=0, sticky="w")
        mode_menu = tk.OptionMenu(
            self.controls,
            self.mode_var,
            *MODE_NAMES.values(),
            command=self._on_mode_select,
        )
        mode_menu.configure(bg=BG, fg=label_fg, activebackground=BG, highlightthickness=0)
        mode_menu.grid(row=0, column=1, padx=(6, 16), sticky="w")

        # Slowdown threshold slider.
        self.slow_var = tk.DoubleVar(value=self.slow_dist)
        slow_label = tk.Label(self.controls, text="Slow distance: start decelerating", bg=BG, fg=label_fg)
        slow_label.grid(row=0, column=2, sticky="w")
        slow_scale = tk.Scale(
            self.controls,
            variable=self.slow_var,
            from_=5,
            to=260,
            resolution=1,
            orient="horizontal",
            length=200,
            command=self._on_slow_change,
            bg=BG,
            fg=label_fg,
            highlightthickness=0,
        )
        slow_scale.grid(row=0, column=3, padx=(6, 16), sticky="w")

        # Full-stop threshold slider.
        self.stop_var = tk.DoubleVar(value=self.stop_dist)
        stop_label = tk.Label(self.controls, text="Stop distance: full stop threshold", bg=BG, fg=label_fg)
        stop_label.grid(row=0, column=4, sticky="w")
        stop_scale = tk.Scale(
            self.controls,
            variable=self.stop_var,
            from_=1,
            to=80,
            resolution=1,
            orient="horizontal",
            length=160,
            command=self._on_stop_change,
            bg=BG,
            fg=label_fg,
            highlightthickness=0,
        )
        stop_scale.grid(row=0, column=5, padx=(6, 16), sticky="w")

        # Repulsion gain slider for model 2.
        self.repel_var = tk.DoubleVar(value=self.repel_k)
        repel_label = tk.Label(self.controls, text="Repulsion strength (Model 2)", bg=BG, fg=label_fg)
        repel_label.grid(row=0, column=6, sticky="w")
        repel_scale = tk.Scale(
            self.controls,
            variable=self.repel_var,
            from_=0,
            to=12000,
            resolution=100,
            orient="horizontal",
            length=200,
            command=self._on_repel_change,
            bg=BG,
            fg=label_fg,
            highlightthickness=0,
        )
        repel_scale.grid(row=0, column=7, padx=(6, 0), sticky="w")

    def _on_mode_select(self, _value):
        """Mode dropdown callback. Input: selected label string (unused directly)."""
        # Map selected mode label back to its numeric id.
        for k, v in MODE_NAMES.items():
            if v == self.mode_var.get():
                self.mode = k
                return

    def _on_slow_change(self, _value):
        """Slow-distance slider callback. Input: slider value string/number."""
        # Keep slow_dist safely above stop_dist.
        self.slow_dist = max(self.stop_dist + 5.0, float(self.slow_var.get()))
        self.slow_var.set(self.slow_dist)

    def _on_stop_change(self, _value):
        """Stop-distance slider callback. Input: slider value string/number."""
        # Keep stop_dist positive and preserve slow > stop.
        self.stop_dist = max(1.0, float(self.stop_var.get()))
        self.slow_dist = max(self.stop_dist + 5.0, self.slow_dist)
        self.slow_var.set(self.slow_dist)

    def _on_repel_change(self, _value):
        """Repulsion slider callback. Input: slider value string/number."""
        self.repel_k = max(0.0, float(self.repel_var.get()))

    def _init_scene(self):
        """Reset obstacle scene and place the dot. No external inputs."""
        # Build boundary walls and random interior obstacles.
        self.obstacles = [
            WallObstacle("left"),
            WallObstacle("right"),
            WallObstacle("top"),
            WallObstacle("bottom"),
        ]
        self.spawn_rects()
        self.spawn_random_circles(6)
        self.place_dot()

    def spawn_random_circles(self, n):
        """Spawn up to n random circles while respecting clearance."""
        # Try repeatedly to place non-overlapping circles.
        for _ in range(n):
            placed = False
            for _attempt in range(200):
                r = random.randint(20, 60)
                x = random.randint(r + 50, WIDTH - r - 50)
                y = random.randint(r + 50, HEIGHT - r - 50)
                if self.can_place_circle((x, y), r):
                    self.obstacles.append(CircleObstacle(x, y, r))
                    placed = True
                    break
            if not placed:
                break

    def can_place_circle(self, center, r):
        """Check if a circle fits. Inputs: center=(x, y), radius r."""
        # A circle is valid if it has at least CLEARANCE from every obstacle.
        for obs in self.obstacles:
            if distance_to_obstacle(center, obs, r) < CLEARANCE:
                return False
        return True

    def place_dot(self):
        """Place the player dot in free space. No external inputs."""
        # Spawn the agent in free space; fallback to center if needed.
        for _attempt in range(500):
            x = random.randint(DOT_R + 10, WIDTH - DOT_R - 10)
            y = random.randint(DOT_R + 10, HEIGHT - DOT_R - 10)
            if self.is_free((x, y), DOT_R):
                self.dot[0] = x
                self.dot[1] = y
                return
        # fallback: center if all attempts failed
        self.dot[0] = WIDTH * 0.5
        self.dot[1] = HEIGHT * 0.5

    def is_free(self, p, r):
        """Check if point p=(x, y) with radius r is collision-free."""
        # Generic free-space query for a point with radius r.
        for obs in self.obstacles:
            if distance_to_obstacle(p, obs, r) < CLEARANCE:
                return False
        return True

    def spawn_rects(self):
        """Add fixed rectangle and segment obstacles. No external inputs."""
        # table-like blocks
        self.obstacles.append(RectObstacle(120, 380, 240, 40))
        self.obstacles.append(RectObstacle(520, 120, 220, 40))
        self.obstacles.append(RectObstacle(420, 260, 40, 200))
        # line segments as thin rails
        self.obstacles.append(SegmentObstacle(80, 140, 300, 140))
        self.obstacles.append(SegmentObstacle(600, 460, 820, 520))
        # additional diagonal lines
        self.obstacles.append(SegmentObstacle(200, 50, 400, 250))
        self.obstacles.append(SegmentObstacle(650, 300, 750, 100))
        self.obstacles.append(SegmentObstacle(100, 480, 350, 350))

    def on_key_down(self, e):
        """Key-press handler. Input: Tk event e with e.keysym."""
        # Track currently pressed keys for continuous movement.
        self.keys.add(e.keysym)
        if e.keysym in ["1", "2", "3", "4"]:
            self.mode = int(e.keysym)
            self.mode_var.set(MODE_NAMES[self.mode])
        if e.keysym in ["r", "R"]:
            self._init_scene()
        # parameter tuning
        if e.keysym in ["bracketleft"]:
            self.adjust_params(-1)
        if e.keysym in ["bracketright"]:
            self.adjust_params(1)
        if e.keysym in ["minus", "underscore"]:
            self.adjust_stop(-1)
        if e.keysym in ["equal", "plus"]:
            self.adjust_stop(1)
        if e.keysym in ["comma"]:
            self.adjust_repulse(-1)
        if e.keysym in ["period"]:
            self.adjust_repulse(1)

    def on_key_up(self, e):
        """Key-release handler. Input: Tk event e with e.keysym."""
        if e.keysym in self.keys:
            self.keys.remove(e.keysym)

    def adjust_params(self, sign):
        """Adjust slow distance. Input: sign is +1 or -1."""
        # Increase/decrease slow distance via keyboard.
        self.slow_dist = max(self.stop_dist + 5.0, self.slow_dist + sign * PARAM_STEP)
        self.slow_var.set(self.slow_dist)

    def adjust_stop(self, sign):
        """Adjust stop distance. Input: sign is +1 or -1."""
        self.stop_dist = max(1.0, self.stop_dist + sign * PARAM_STEP)
        self.slow_dist = max(self.stop_dist + 5.0, self.slow_dist)
        self.stop_var.set(self.stop_dist)
        self.slow_var.set(self.slow_dist)

    def adjust_repulse(self, sign):
        """Adjust repulsion gain. Input: sign is +1 or -1."""
        self.repel_k = max(0.0, self.repel_k + sign * REPEL_STEP)
        self.repel_var.set(self.repel_k)

    def compute_cmd(self):
        """Compute acceleration command from key state. No external inputs."""
        # Convert key state into a normalized acceleration direction.
        ax = 0.0
        ay = 0.0
        if "Left" in self.keys or "a" in self.keys or "A" in self.keys:
            ax -= 1.0
        if "Right" in self.keys or "d" in self.keys or "D" in self.keys:
            ax += 1.0
        if "Up" in self.keys or "w" in self.keys or "W" in self.keys:
            ay -= 1.0
        if "Down" in self.keys or "s" in self.keys or "S" in self.keys:
            ay += 1.0

        a = vec_norm((ax, ay))
        # Scale unit direction by acceleration limit.
        self.cmd[0] = a[0] * ACCEL
        self.cmd[1] = a[1] * ACCEL

    def nearest_obstacle(self):
        """Return nearest obstacle data: (distance d, normal n)."""
        # Return closest signed distance and its surface normal.
        p = (self.dot[0], self.dot[1])
        best_d = 1e9
        best_n = (0.0, 0.0)
        for obs in self.obstacles:
            d, n = obs.dist_and_normal(p)
            if d < best_d:
                best_d = d
                best_n = n
        return best_d, best_n

    def repulsive_field(self):
        """Compute summed repulsion and nearest obstacle info for current dot position."""
        # Sum obstacle repulsion forces, and also track nearest obstacle.
        p = (self.dot[0], self.dot[1])
        total = (0.0, 0.0)
        min_d = 1e9
        min_n = (0.0, 0.0)
        for obs in self.obstacles:
            d, n = obs.dist_and_normal(p)
            if d < min_d:
                min_d = d
                min_n = n
            if d <= 0:
                # Hard push while penetrating.
                push = vec_mul(n, REPEL_MAX)
                total = vec_add(total, push)
                continue
            if d < REPEL_DIST:
                # Inverse-distance repulsion with magnitude cap.
                mag = self.repel_k * (1.0 / d - 1.0 / REPEL_DIST) / (d * d)
                mag = clamp(mag, 0.0, REPEL_MAX)
                total = vec_add(total, vec_mul(n, mag))
        return total, min_d, min_n

    def _correct_position(self):
        """Push dot out of penetrated obstacles to prevent clipping.

        Inputs: none (uses current self.dot and self.obstacles).
        
        Iterates through all obstacles and checks for penetration (distance < 0).
        If penetrated, pushes the dot back out along the surface normal.
        Multiple iterations handle concave corners where multiple obstacles
        may constrain the dot simultaneously.
        """
        p = [self.dot[0], self.dot[1]]
        
        # Multiple iterations to handle corners with multiple touching obstacles
        for _iteration in range(3):
            for obs in self.obstacles:
                d, n = obs.dist_and_normal(p)
                if d < 0:  # Penetrated
                    # Push dot back to surface plus small clearance
                    push_dist = -d + 0.1
                    p[0] += n[0] * push_dist
                    p[1] += n[1] * push_dist
        
        self.dot[0] = p[0]
        self.dot[1] = p[1]

    def apply_model(self, d, n, repulse):
        """Apply active control model.

        Inputs:
        d: nearest signed distance to obstacle
        n: nearest obstacle normal (unit vector)
        repulse: summed repulsive velocity/force-like vector
        """
        # Start from current velocity command, then modify by selected model.
        v_cmd = (self.vel[0], self.vel[1])
        v_next = v_cmd

        if self.mode == 1:
            # Scale speed down as distance approaches stop threshold.
            s = clamp((d - self.stop_dist) / max(1e-6, (self.slow_dist - self.stop_dist)), 0.0, 1.0)
            v_next = vec_mul(v_cmd, s)

        elif self.mode == 2:
            # Add synthesized repulsive field directly to velocity.
            v_next = vec_add(v_cmd, repulse)

        elif self.mode == 3:
            # PROJECT NORMAL mode: Remove only the velocity component directed into obstacles
            # Apply projection against ALL obstacles within slow_dist for proper corner handling
            # 
            # This ensures that at concave corners (e.g., between two walls), projections
            # from both obstacles are applied, preventing clipping through either surface.
            # 
            # For each nearby obstacle:
            # - Compute the surface normal vector 'n' pointing outward from the obstacle
            # - Calculate into = -dot(v_next, n): magnitude of motion TOWARD the obstacle
            # - If into > 0, the agent is moving into the obstacle
            # - Apply correction: v_next += n * into, canceling the inward component
            # 
            # Result: Smooth tangential sliding along surfaces, proper multi-surface handling,
            # no barrier effect at distance (only activates within slow_dist)
            v_next = v_cmd
            p = (self.dot[0], self.dot[1])
            for obs in self.obstacles:
                d_obs, n_obs = obs.dist_and_normal(p)
                if d_obs < self.slow_dist:  # Only activate when within detection range
                    into = -vec_dot(v_next, n_obs)
                    if into > 0:
                        v_next = vec_add(v_next, vec_mul(n_obs, into))

        elif self.mode == 4:
            # Dampen only the velocity component along obstacle normal.
            s = clamp((d - self.stop_dist) / max(1e-6, (self.slow_dist - self.stop_dist)), 0.0, 1.0)
            vn = vec_mul(n, vec_dot(v_cmd, n))
            vt = vec_sub(v_cmd, vn)
            v_next = vec_add(vec_mul(vn, s), vt)

        speed = vec_len(v_next)
        if speed > MAX_SPEED:
            # Enforce global speed cap.
            v_next = vec_mul(vec_norm(v_next), MAX_SPEED)
        return v_next

    def update_physics(self):
        """Advance simulation one fixed timestep. No external inputs."""
        # Recompute commanded acceleration from current key state.
        self.compute_cmd()

        # integrate joystick accel into velocity command
        self.vel[0] += self.cmd[0] * DT
        self.vel[1] += self.cmd[1] * DT

        # soft friction when no input
        if vec_len(self.cmd) < 1e-6:
            self.vel[0] *= 0.92
            self.vel[1] *= 0.92

        repulse, d, n = self.repulsive_field()
        # Apply selected collision-avoidance model to produce next velocity.
        v_next = self.apply_model(d, n, repulse)

        self.dot[0] += v_next[0] * DT
        self.dot[1] += v_next[1] * DT

        # Correct position if penetrated any obstacles
        self._correct_position()

        # keep inside bounds to avoid drift
        self.dot[0] = clamp(self.dot[0], DOT_R, WIDTH - DOT_R)
        self.dot[1] = clamp(self.dot[1], DOT_R, HEIGHT - DOT_R)

        # store actual velocity for next step
        self.vel[0] = v_next[0]
        self.vel[1] = v_next[1]

    def draw(self):
        """Render current frame to canvas. No external inputs."""
        # Clear and redraw entire frame.
        self.canvas.delete("all")

        # Draw obstacles with type-specific primitives.
        for obs in self.obstacles:
            if isinstance(obs, CircleObstacle):
                x0 = obs.x - obs.r
                y0 = obs.y - obs.r
                x1 = obs.x + obs.r
                y1 = obs.y + obs.r
                self.canvas.create_oval(x0, y0, x1, y1, outline=FG, width=2)
            if isinstance(obs, RectObstacle):
                self.canvas.create_rectangle(
                    obs.x,
                    obs.y,
                    obs.x + obs.w,
                    obs.y + obs.h,
                    outline=FG,
                    width=2,
                )
            if isinstance(obs, SegmentObstacle):
                self.canvas.create_line(obs.ax, obs.ay, obs.bx, obs.by, fill=FG, width=2)

        self.canvas.create_oval(
            self.dot[0] - DOT_R,
            self.dot[1] - DOT_R,
            self.dot[0] + DOT_R,
            self.dot[1] + DOT_R,
            fill=ACCENT,
            outline="",
        )

        d, _n = self.nearest_obstacle()
        # HUD with mode and controls.
        info = f"Mode {self.mode}: {MODE_NAMES[self.mode]}   |   d = {d:.1f}"
        help1 = "Move: Arrows/WASD   |   Randomize: R"
        self.canvas.create_text(12, 12, anchor="nw", fill=FG, text=info, font=("Helvetica", 12))
        self.canvas.create_text(12, 32, anchor="nw", fill=FG, text=help1, font=("Helvetica", 11))

        if d < self.stop_dist:
            self.canvas.create_text(12, 52, anchor="nw", fill=WARN, text="Contact zone", font=("Helvetica", 11))

    def tick(self):
        """Main loop callback called once per frame. No external inputs."""
        # Main fixed-step loop: simulate, log, render, schedule next frame.
        self.update_physics()
        # log
        d, _n = self.nearest_obstacle()
        speed = vec_len(self.vel)
        cmd_speed = vec_len(self.cmd)
        self.log_file.write(
            f"{self.t:.3f},{self.mode},{d:.3f},{speed:.3f},{cmd_speed:.3f},{self.dot[0]:.2f},{self.dot[1]:.2f}\n"
        )
        self.t += DT
        self.draw()
        self.root.after(int(1000 / FPS), self.tick)

    def on_close(self):
        """Window-close handler. No inputs; flushes log and destroys root."""
        try:
            self.log_file.flush()
            self.log_file.close()
        finally:
            self.root.destroy()


if __name__ == "__main__":
    # Fail fast with a helpful message if Tk support is missing.
    if tk is None:
        raise RuntimeError("tkinter is not available. Please install Tk support for Python.")
    root = tk.Tk()
    root.title("Collision-Aware Joystick Mockup")
    root.resizable(False, False)
    Game(root)
    root.mainloop()
