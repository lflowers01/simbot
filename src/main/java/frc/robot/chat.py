"""
bluey_basket_full.py

Full toddler-friendly basketball arcade (touch + 12 buttons).
- File-based sounds only; missing sound files -> silent.
- Background music (music.mp3) if present.
- Balls respawn after random 3-7s and cannot auto-shoot.
- Character images animate on shoot/score/miss with voice lines.
- Miss chance configurable.
"""

import pygame
import os
import random
import math
import time
import sys
import platform

# Try to import HID library for better device detection
try:
    import hid

    HID_AVAILABLE = True
    print("HID library available for enhanced device detection")
except ImportError:
    HID_AVAILABLE = False
    print("HID library not available, using pygame joystick detection only")

# ----------------------------
# CONFIG CONSTANTS (EDIT HERE)
# ----------------------------

# Reference resolution for scaling calculations
REFERENCE_W = 1280
REFERENCE_H = 720
FPS = 60

# Images (set paths or leave None to use drawn fallback)
BALL_IMAGE_PATH = "ball.png"  # basketball image (transparent PNG)
CENTER_HOOP_IMAGE_PATH = "hoop.png"  # center hoop/backboard image
EDGE_HOOP_IMAGE_PATH = "hoop_small.png"  # small hoop for edge pads (optional)
PLAYER_IMAGE_PATHS = [
    "chilli.png",  # player 0 (left)
    "bluey.png",  # player 1 (bottom)
    "bingo.png",  # player 2 (right)
]

# Sounds (file paths). If missing -> silent (no crash).
SOUND_SHOOT_FILE = "shot.wav"  # when a pad shoots (whoosh)
SOUND_SWISH_FILE = "swish.wav"  # net swish (good score)
SOUND_SCORE_FILE = "score.wav"  # celebratory sound on score
SOUND_MISS_FILE = "miss.wav"  # optional miss/rim sound

# Voice lines per character (list of lists). Can be empty lists.
VOICE_FILES_PER_PLAYER = [
    ["haw.mp3", ""],  # player 0 voice files
    ["boohoo.mp3"],  # player 1 voice files
    ["rage.mp3"],  # player 2 voice files
]

# Background music (optional)
BACKGROUND_MUSIC = "music.mp3"

# Gameplay tuning
TOTAL_BUTTONS = 12
PAD_MARGIN = 50  # Increased margin for larger assets
PAD_RADIUS = 94  # Increased by 20%
BALL_BASE_RADIUS = int(PAD_RADIUS * 0.65)
BALL_RESPAWN_MIN = 1.0  # seconds
BALL_RESPAWN_MAX = 5.0  # seconds
MISS_CHANCE = 0.25  # chance to miss (0..1)
ATTRACT_AFTER = float("inf")  # Disable attract mode by setting an unreachable timeout

# Keyboard fallback -> maps some keys to buttons 0..11 for testing
KEYBOARD_TO_BUTTON = {
    pygame.K_q: 0,
    pygame.K_w: 1,
    pygame.K_e: 2,
    pygame.K_r: 3,
    pygame.K_a: 4,
    pygame.K_s: 5,
    pygame.K_d: 6,
    pygame.K_f: 7,
    pygame.K_u: 8,
    pygame.K_i: 9,
    pygame.K_o: 10,
    pygame.K_p: 11,
}

# Debugging HID mapping prints (set True to print joystick button indices)
DEBUG_PRINT_JOY = False

# ----------------------------
# END CONFIG
# ----------------------------

# Initialize pygame mixer for Raspberry Pi compatibility
pygame.mixer.pre_init(frequency=22050, size=-16, channels=2, buffer=1024)
pygame.init()

# Get actual screen dimensions - use pygame's scaling instead
display_info = pygame.display.Info()
SCREEN_W = display_info.current_w
SCREEN_H = display_info.current_h

print(f"Detected screen resolution: {SCREEN_W}x{SCREEN_H}")

# Use native fullscreen resolution - no scaling
screen = pygame.display.set_mode((0, 0), pygame.FULLSCREEN)
pygame.display.set_caption("Bluey Basketball — Full")
clock = pygame.time.Clock()

# Colors
BG_COLOR = pygame.Color(18, 24, 46)
PLAYER_COLORS = [
    pygame.Color(111, 182, 255),
    pygame.Color(255, 183, 112),
    pygame.Color(172, 255, 157),
]
ACCENT_COLS = [
    pygame.Color(255, 110, 199),
    pygame.Color(255, 233, 84),
    pygame.Color(170, 120, 255),
]
WHITE = (255, 255, 255)
RIM_COLOR = (180, 30, 30)

# Update player colors for pads
PLAYER_PAD_COLORS = [
    (255, 0, 0),  # Red for Player 0
    (0, 255, 0),  # Green for Player 1
    (0, 0, 255),  # Blue for Player 2
    (255, 255, 0),  # Yellow for Player 3
]

# Update player pad colors so each player has red, blue, yellow, and green pads
PAD_COLORS = [
    (255, 0, 0),  # Red
    (0, 0, 255),  # Blue
    (255, 255, 0),  # Yellow
    (0, 255, 0),  # Green
]

# Simplified constants - scaled up by 20%
PAD_MARGIN = 50
PAD_RADIUS = 94
BALL_BASE_RADIUS = int(PAD_RADIUS * 0.65)

# Safe shutdown state
shutdown_confirmation_time = None
SHUTDOWN_CONFIRMATION_DELAY = 2.0  # Hold button for 2 seconds to confirm

# ----------------------------
# Asset reloading system
# ----------------------------
last_asset_reload = 0
ASSET_RELOAD_INTERVAL = 3.0  # Reload assets every 3 seconds


# ----------------------------
# Safe asset loaders with retry logic
# ----------------------------
def load_image_with_retry(path, scale=None, max_attempts=3):
    """Load image with retry logic for better reliability and maintain aspect ratio"""
    if not path or not isinstance(path, str):
        return None

    for attempt in range(max_attempts):
        if os.path.exists(path):
            try:
                im = pygame.image.load(path).convert_alpha()
                if scale and len(scale) == 2 and scale[0] > 0 and scale[1] > 0:
                    original_width, original_height = im.get_width(), im.get_height()
                    target_width, target_height = scale

                    # Prevent division by zero
                    if original_height == 0 or original_width == 0:
                        print(f"Warning: Image {path} has zero dimensions")
                        return None

                    aspect_ratio = original_width / original_height

                    # Adjust scale to maintain aspect ratio
                    if target_width / target_height > aspect_ratio:
                        target_width = int(target_height * aspect_ratio)
                    else:
                        target_height = int(target_width / aspect_ratio)

                    # Ensure minimum size of 1x1
                    target_width = max(1, target_width)
                    target_height = max(1, target_height)

                    im = pygame.transform.smoothscale(im, (target_width, target_height))
                print(f"Successfully loaded image: {path}")
                return im
            except (pygame.error, OSError, MemoryError) as e:
                print(f"Attempt {attempt + 1} failed to load image {path}: {e}")
                if attempt < max_attempts - 1:
                    time.sleep(0.5)  # Brief delay before retry
                continue
        else:
            print(f"Image file not found: {path}")
            break
    return None


def load_sound_with_retry(path, max_attempts=3):
    """Load sound with retry logic for better reliability"""
    if not path or not isinstance(path, str):
        return None

    for attempt in range(max_attempts):
        if os.path.exists(path):
            try:
                sound = pygame.mixer.Sound(path)
                # Validate sound object
                if sound.get_length() <= 0:
                    print(f"Warning: Sound {path} has zero length")
                    return None
                print(f"Successfully loaded sound: {path}")
                return sound
            except (pygame.error, OSError, MemoryError) as e:
                print(f"Attempt {attempt + 1} failed to load sound {path}: {e}")
                if attempt < max_attempts - 1:
                    time.sleep(0.5)  # Brief delay before retry
                continue
        else:
            print(f"Sound file not found: {path}")
            break
    return None


def load_image(path, scale=None):
    return load_image_with_retry(path, scale)


def load_sound(path):
    return load_sound_with_retry(path)


# Ensure ShootingBall class is defined before it is used
class ShootingBall:
    def __init__(self):
        self.pad_idx = None  # Assigned randomly after cooldown
        self.pad_pos = None
        self.current_pos = None
        self.radius = max(1, BALL_BASE_RADIUS)  # Ensure minimum radius
        self.bounce_phase = random.uniform(0, 2 * math.pi)
        self.has_ball = False  # Start without a ball
        self.respawn_timer = random.uniform(
            max(0.1, BALL_RESPAWN_MIN), max(0.1, BALL_RESPAWN_MAX)
        )  # Start with cooldown
        self.state = "cooldown"  # cooldown, idle, or shooting
        self.vx = 0.0
        self.vy = 0.0
        self.intended_miss = False
        self.image = None
        self.spawn_scale = 0.0  # Scale for spawn-in animation
        self.update_ball_image()

    def update_ball_image(self):
        """Update ball image when assets are reloaded"""
        self.image = None
        if ball_image and self.radius > 0:
            try:
                size = max(1, self.radius * 2)
                self.image = pygame.transform.smoothscale(ball_image, (size, size))
            except (pygame.error, MemoryError) as e:
                print(f"Failed to scale ball image: {e}")
                self.image = None

    def assign_random_pad(self, active_pads):
        try:
            # Validate active_pads
            if not isinstance(active_pads, (set, list)):
                active_pads = set()

            available_pads = [i for i in range(TOTAL_BUTTONS) if i not in active_pads]
            if available_pads:
                self.pad_idx = random.choice(available_pads)
                self.pad_pos = get_button_pos(self.pad_idx)
                if self.pad_pos:  # Ensure pad_pos is valid
                    self.current_pos = list(self.pad_pos)
                else:
                    print(f"Warning: Invalid pad position for pad {self.pad_idx}")
                    self.pad_idx = None
        except Exception as e:
            print(f"Error assigning pad: {e}")
            self.pad_idx = None

    def update(self, dt, active_pads):
        try:
            # Clamp dt to prevent physics instability and validate
            dt = max(0, min(dt, 0.1))
            if dt == 0:
                return

            if self.state == "cooldown":
                self.respawn_timer -= dt
                if self.respawn_timer <= 0:
                    self.assign_random_pad(active_pads)
                    if self.pad_idx is not None:  # Only proceed if pad was assigned
                        self.has_ball = True
                        self.state = "spawning"
                        self.spawn_scale = 0.0
            elif self.state == "spawning":
                self.spawn_scale += dt * 2.5  # Grow scale over time
                if self.spawn_scale >= 1.0:
                    self.spawn_scale = 1.0
                    self.state = "idle"
            elif self.state == "idle":
                if self.current_pos and self.pad_pos:  # Ensure current_pos exists
                    self.bounce_phase += dt * 3.2
                    offset = math.sin(self.bounce_phase) * (self.radius * 0.15)
                    self.current_pos[0] = self.pad_pos[0]
                    self.current_pos[1] = self.pad_pos[1] + int(offset)
            elif self.state == "shooting":
                if self.current_pos:  # Ensure current_pos exists
                    self.vy += 420 * dt
                    self.current_pos[0] += self.vx * dt
                    self.current_pos[1] += self.vy * dt

                    # Bounds checking for screen edges
                    if (
                        self.current_pos[0] < -100
                        or self.current_pos[0] > SCREEN_W + 100
                        or self.current_pos[1] > SCREEN_H + 300
                    ):
                        self.reset_after_shot()
                        return

                    tx, ty = CENTER_POS
                    dx = self.current_pos[0] - tx
                    dy = self.current_pos[1] - ty
                    if math.hypot(dx, dy) < 28 or self.current_pos[1] > SCREEN_H + 300:
                        if self.intended_miss:
                            self.on_miss()
                        else:
                            self.on_score()
                        self.reset_after_shot()
        except Exception as e:
            print(f"Error updating ball: {e}")
            self.reset_after_shot()

    def draw(self, surf):
        if self.state == "cooldown" or not self.current_pos:
            return
        cx, cy = int(self.current_pos[0]), int(self.current_pos[1])
        scale = self.spawn_scale if self.state == "spawning" else 1.0
        if self.image:
            r = int(self.radius * scale)
            try:
                scaled_image = pygame.transform.smoothscale(self.image, (r * 2, r * 2))
                surf.blit(scaled_image, (cx - r, cy - r))
            except Exception:
                # Fallback to drawn ball if image scaling fails
                self.draw_fallback_ball(surf, cx, cy, scale)
        else:
            self.draw_fallback_ball(surf, cx, cy, scale)

    def draw_fallback_ball(self, surf, cx, cy, scale):
        """Draw fallback basketball when image is unavailable"""
        r = int(self.radius * scale)
        pygame.draw.circle(surf, (239, 118, 33), (cx, cy), r)
        pygame.draw.line(
            surf,
            (40, 30, 30),
            (cx - r // 2, cy),
            (cx + r // 2, cy),
            max(1, int(3 * scale)),
        )

    def shoot_to_center(self):
        try:
            if not self.has_ball or self.state == "shooting" or not self.current_pos:
                # Create particles even for failed shots for visual feedback
                if self.current_pos:
                    cx, cy = int(self.current_pos[0]), int(self.current_pos[1])
                    for _ in range(8):
                        try:
                            particles.append(
                                Particle(
                                    (
                                        cx + random.uniform(-8, 8),
                                        cy + random.uniform(-8, 8),
                                    ),
                                    random.choice(PLAYER_COLORS),
                                )
                            )
                        except Exception:
                            pass  # Skip if particle creation fails
                if SND_SHOOT:
                    try:
                        SND_SHOOT.play()
                    except pygame.error:
                        pass
                return

            set_last_input_time()
            will_miss = random.random() < max(0.0, min(1.0, MISS_CHANCE))
            self.intended_miss = will_miss
            tx, ty = CENTER_POS
            if will_miss:
                tx += random.uniform(-120, 120)
                ty += random.uniform(-30, 60)
            sx, sy = self.current_pos
            travel_time = max(
                0.1, 0.7 + random.random() * 0.35
            )  # Ensure minimum travel time

            # Prevent division by zero
            if travel_time > 0:
                self.vx = (tx - sx) / travel_time
                g = 420.0
                self.vy = (ty - sy - 0.5 * g * travel_time**2) / travel_time
            else:
                self.vx = 0
                self.vy = -200  # Default upward velocity

            self.state = "shooting"
            self.has_ball = False

            # Safe function calls
            try:
                animate_player(btn_to_player(self.pad_idx), "shoot")
            except Exception:
                pass

            if SND_SHOOT:
                try:
                    SND_SHOOT.play()
                except pygame.error:
                    pass

            try:
                play_random_voice(btn_to_player(self.pad_idx))
            except Exception:
                pass

        except Exception as e:
            print(f"Error in shoot_to_center: {e}")

    def on_score(self):
        cx, cy = CENTER_POS
        for _ in range(20):
            particles.append(
                Particle(
                    (cx + random.uniform(-30, 30), cy + random.uniform(-10, 30)),
                    pygame.Color(239, 118, 33),
                )
            )
        if SND_SWISH:
            SND_SWISH.play()
        if SND_SCORE:
            SND_SCORE.play()
        play_random_voice(btn_to_player(self.pad_idx))
        animate_player(btn_to_player(self.pad_idx), "score")
        success_bursts.append(
            (btn_to_player(self.pad_idx), (cx, cy), pygame.time.get_ticks())
        )

    def on_miss(self):
        cx, cy = int(self.current_pos[0]), int(self.current_pos[1])
        for _ in range(12):
            particles.append(
                Particle(
                    (cx + random.uniform(-22, 22), cy + random.uniform(-6, 22)),
                    pygame.Color(200, 200, 200),
                )
            )
        if SND_MISS:
            SND_MISS.play()
        play_random_voice(btn_to_player(self.pad_idx))
        animate_player(btn_to_player(self.pad_idx), "miss")

    def reset_after_shot(self):
        self.state = "cooldown"
        self.has_ball = False
        self.respawn_timer = random.uniform(BALL_RESPAWN_MIN, BALL_RESPAWN_MAX)
        self.pad_idx = None
        self.pad_pos = None
        self.current_pos = None
        self.intended_miss = False


# Initialize global asset variables before first use
ball_image = None
center_hoop_image = None
edge_hoop_image = None
player_images = [None, None, None]
SND_SHOOT = None
SND_SWISH = None
SND_SCORE = None
SND_MISS = None
VOICE_SOUNDS = []


# Modify the `reload_all_assets` function to handle missing assets gracefully
def reload_all_assets():
    global ball_image, center_hoop_image, edge_hoop_image, player_images
    global SND_SHOOT, SND_SWISH, SND_SCORE, SND_MISS, VOICE_SOUNDS

    print("Reloading all game assets...")

    try:
        # Reload images with direct sizing (with bounds checking)
        ball_size = max(1, BALL_BASE_RADIUS * 2)
        hoop_size = max(1, CENTER_RIM_RADIUS * 2)

        ball_image = load_image(BALL_IMAGE_PATH, (ball_size, ball_size))
        center_hoop_image = load_image(
            CENTER_HOOP_IMAGE_PATH, (hoop_size * 2, hoop_size * 2)
        )
        edge_hoop_image = load_image(EDGE_HOOP_IMAGE_PATH, (hoop_size, hoop_size))

        # Safely load player images
        new_player_images = []
        for p in PLAYER_IMAGE_PATHS:
            img = load_image(p, (144, 144))
            new_player_images.append(img)
        player_images = new_player_images

        # Reload sounds with volume adjustment
        SND_SHOOT = load_sound(SOUND_SHOOT_FILE)
        SND_SWISH = load_sound(SOUND_SWISH_FILE)
        SND_SCORE = load_sound(SOUND_SCORE_FILE)
        SND_MISS = load_sound(SOUND_MISS_FILE)

        # Set volumes for better audio balance (with validation)
        sounds_to_set = [
            (SND_SHOOT, 0.7),
            (SND_SWISH, 0.8),
            (SND_SCORE, 0.9),
            (SND_MISS, 0.6),
        ]

        for sound, volume in sounds_to_set:
            if sound:
                try:
                    sound.set_volume(max(0.0, min(1.0, volume)))
                except pygame.error as e:
                    print(f"Failed to set volume for sound: {e}")

        # Reload voice lists (with bounds checking)
        VOICE_SOUNDS = []
        for i, plist in enumerate(VOICE_FILES_PER_PLAYER):
            if not isinstance(plist, (list, tuple)):
                print(f"Warning: Voice files for player {i} is not a list")
                VOICE_SOUNDS.append([])
                continue

            loaded = []
            for p in plist:
                if p and isinstance(p, str):  # Skip empty strings and non-strings
                    s = load_sound(p)
                    if s:
                        try:
                            s.set_volume(0.8)
                            loaded.append(s)
                        except pygame.error as e:
                            print(f"Failed to set volume for voice sound: {e}")
            VOICE_SOUNDS.append(loaded)

        # Reload background music if not playing (with error handling)
        if (
            BACKGROUND_MUSIC
            and isinstance(BACKGROUND_MUSIC, str)
            and os.path.exists(BACKGROUND_MUSIC)
        ):
            if not pygame.mixer.music.get_busy():
                try:
                    pygame.mixer.music.load(BACKGROUND_MUSIC)
                    pygame.mixer.music.set_volume(0.4)
                    pygame.mixer.music.play(-1)
                    print(f"Background music loaded: {BACKGROUND_MUSIC}")
                except pygame.error as e:
                    print(f"Failed to load background music: {e}")

        print("Asset reload complete")

    except Exception as e:
        print(f"Critical error during asset reload: {e}")
        import traceback

        traceback.print_exc()


# Ensure the `balls` list is initialized after the `ShootingBall` class is defined
balls = [ShootingBall() for _ in range(8)]  # Fixed number of balls

# Initial asset load - ONLY call this once
reload_all_assets()


# Simplified joystick detection - no power button separation needed
def detect_joystick_devices():
    """Detect joystick devices for game input only"""
    joysticks = []
    game_joystick_id = None

    pygame.joystick.init()
    print("Detecting pygame joystick devices...")

    joystick_count = pygame.joystick.get_count()
    if joystick_count == 0:
        print("No joystick devices found!")
        return joysticks, None, game_joystick_id

    # Store device info for decision making
    device_info = []

    for i in range(joystick_count):
        try:
            j = pygame.joystick.Joystick(i)
            j.init()
            joysticks.append(j)

            # Get device information
            button_count = j.get_numbuttons()
            device_name = j.get_name()
            device_guid = j.get_guid()
            axis_count = j.get_numaxes()
            hat_count = j.get_numhats()

            print(f"Pygame Joystick {i}:")
            print(f"  Name: {device_name}")
            print(f"  GUID: {device_guid}")
            print(f"  Buttons: {button_count}")
            print(f"  Axes: {axis_count}")
            print(f"  Hats: {hat_count}")

            device_info.append(
                {
                    "index": i,
                    "name": device_name.lower(),
                    "buttons": button_count,
                    "axes": axis_count,
                    "hats": hat_count,
                    "joystick": j,
                }
            )

        except Exception as e:
            print(f"Failed to initialize joystick {i}: {e}")

    # Find the best game controller
    # Priority 1: Device with exactly TOTAL_BUTTONS or more
    suitable_devices = [d for d in device_info if d["buttons"] >= TOTAL_BUTTONS]
    if suitable_devices:
        # Sort by button count (prefer more buttons) then by index (prefer lower index)
        suitable_devices.sort(key=lambda x: (-x["buttons"], x["index"]))
        game_joystick_id = suitable_devices[0]["index"]
        print(f"  -> GAME CONTROLLER device (best match): Joystick {game_joystick_id}")
        print(
            f"     Device has {suitable_devices[0]['buttons']} buttons (need {TOTAL_BUTTONS})"
        )
    else:
        # Fallback: Use device with most buttons
        if device_info:
            device_info.sort(key=lambda x: (-x["buttons"], x["index"]))
            game_joystick_id = device_info[0]["index"]
            print(
                f"  -> GAME CONTROLLER device (fallback): Joystick {game_joystick_id}"
            )
            print(
                f"     Device has {device_info[0]['buttons']} buttons (need {TOTAL_BUTTONS})"
            )
            if device_info[0]["buttons"] < TOTAL_BUTTONS:
                print(f"     WARNING: Device has fewer buttons than needed!")

    print()
    return joysticks, None, game_joystick_id  # No power joystick needed


# Detect devices using simplified method
joysticks, power_joystick_id, game_joystick_id = detect_joystick_devices()

print(f"Final device assignment:")
print(f"  Game controller device: {game_joystick_id}")
print(f"  Power button: DISABLED")

if game_joystick_id is None:
    print("WARNING: No game controller device found!")
    print("Game buttons will not work (keyboard fallback available).")
else:
    # Validate the selected joystick
    try:
        selected_joystick = joysticks[game_joystick_id]
        actual_buttons = selected_joystick.get_numbuttons()
        print(f"Selected joystick has {actual_buttons} buttons")
        if actual_buttons < TOTAL_BUTTONS:
            print(
                f"WARNING: Only {actual_buttons} buttons available, need {TOTAL_BUTTONS}"
            )
    except (IndexError, AttributeError) as e:
        print(f"Error validating selected joystick: {e}")

# ----------------------------
# Helper geometry / layout
# ----------------------------


def btn_to_player(b):
    """Convert button index to player with bounds checking"""
    if not isinstance(b, int) or b < 0:
        return 0
    return min(2, b // 4)  # Ensure player is 0, 1, or 2


def btn_to_slot(b):
    """Convert button index to slot with bounds checking"""
    if not isinstance(b, int) or b < 0:
        return 0
    return b % 4


def get_button_pos(btn_idx):
    """Get button position with bounds checking"""
    try:
        if not isinstance(btn_idx, int) or btn_idx < 0 or btn_idx >= TOTAL_BUTTONS:
            print(f"Invalid button index: {btn_idx}")
            return None

        player = btn_to_player(btn_idx)
        slot = btn_to_slot(btn_idx)

        # Ensure screen dimensions are valid
        if SCREEN_W <= 0 or SCREEN_H <= 0:
            print("Invalid screen dimensions")
            return (100, 100)  # Fallback position

        if player == 0:  # left vertical top->bottom
            x = PAD_MARGIN + PAD_RADIUS + 22
            mid_y = SCREEN_H // 2
            spacing = 144
            y = mid_y - (1.5 * spacing) + slot * spacing
            # Bounds checking
            x = max(PAD_RADIUS, min(SCREEN_W - PAD_RADIUS, x))
            y = max(PAD_RADIUS, min(SCREEN_H - PAD_RADIUS, y))
            return int(x), int(y)
        elif player == 1:  # bottom horizontal left->right
            y = SCREEN_H - PAD_MARGIN - PAD_RADIUS - 22
            mid_x = SCREEN_W // 2
            spacing = 192
            x = mid_x - (1.5 * spacing) + slot * spacing
            # Bounds checking
            x = max(PAD_RADIUS, min(SCREEN_W - PAD_RADIUS, x))
            y = max(PAD_RADIUS, min(SCREEN_H - PAD_RADIUS, y))
            return int(x), int(y)
        else:  # right vertical top->bottom
            x = SCREEN_W - PAD_MARGIN - PAD_RADIUS - 22
            mid_y = SCREEN_H // 2
            spacing = 144
            y = mid_y - (1.5 * spacing) + slot * spacing
            # Bounds checking
            x = max(PAD_RADIUS, min(SCREEN_W - PAD_RADIUS, x))
            y = max(PAD_RADIUS, min(SCREEN_H - PAD_RADIUS, y))
            return int(x), int(y)
    except Exception as e:
        print(f"Error in get_button_pos: {e}")
        return (100, 100)  # Safe fallback


CENTER_POS = (SCREEN_W // 2, SCREEN_H // 2 - 24)  # Slightly adjusted
CENTER_RIM_RADIUS = 77  # Increased by 20%


# ----------------------------
# Particle (confetti) class
# ----------------------------
class Particle:
    def __init__(self, pos, color):
        self.x, self.y = pos
        self.vx = random.uniform(-220, 220)
        self.vy = random.uniform(-300, -40)
        self.life = random.uniform(0.6, 1.4)
        self.color = color
        self.size = random.uniform(6, 16)

    def update(self, dt):
        self.life -= dt
        self.vy += 360 * dt
        self.x += self.vx * dt
        self.y += self.vy * dt

    def draw(self, surf):
        if self.life <= 0:
            return
        a = int(255 * max(0, min(1, self.life / 1.4)))
        s = pygame.Surface((int(self.size * 2), int(self.size * 2)), pygame.SRCALPHA)
        pygame.draw.circle(
            s,
            (self.color.r, self.color.g, self.color.b),
            (int(self.size), int(self.size)),
            int(self.size),
        )
        s.set_alpha(a)
        surf.blit(s, (self.x - self.size, self.y - self.size))


# ----------------------------
# Player animation state
# ----------------------------
player_anim = [
    {"scale": 1.0, "timer": 0.0, "state": None, "target": 1.0} for _ in range(3)
]


def animate_player(player_index, action):
    if not (0 <= player_index < 3):
        return
    st = player_anim[player_index]
    st["state"] = action
    st["timer"] = 0.0
    if action == "shoot":
        st["target"] = 1.22
    elif action == "score":
        st["target"] = 1.45
    elif action == "miss":
        st["target"] = 1.12
    elif action == "return":
        st["target"] = 1.08
    else:
        st["target"] = 1.12


def update_player_animations(dt):
    for st in player_anim:
        if st["state"] is None:
            st["scale"] += (1.0 - st["scale"]) * min(1.0, dt * 6.0)
            continue
        st["timer"] += dt
        if st["timer"] < 0.18:
            t = st["timer"] / 0.18
            st["scale"] = 1.0 + (st["target"] - 1.0) * (1 - (1 - t) * (1 - t))
        else:
            st["scale"] += (1.0 - st["scale"]) * min(1.0, dt * 6.0)
            if abs(st["scale"] - 1.0) < 0.01:
                st["scale"] = 1.0
                st["state"] = None
                st["timer"] = 0.0


# ----------------------------
# Voice playback helper
# ----------------------------
def play_random_voice(player_index):
    """Play random voice with bounds checking"""
    try:
        if (
            isinstance(player_index, int)
            and 0 <= player_index < len(VOICE_SOUNDS)
            and VOICE_SOUNDS[player_index]
        ):

            s = random.choice(VOICE_SOUNDS[player_index])
            if s:
                s.play()
    except (pygame.error, IndexError, TypeError) as e:
        print(f"Error playing voice: {e}")


# ----------------------------
# Success bursts & particles
# ----------------------------
particles = []
success_bursts = []


def render_success_bursts(surf):
    now = pygame.time.get_ticks()
    for entry in list(success_bursts):
        player, pos, t = entry
        age = (now - t) / 1000.0
        if age > 1.1:
            try:
                success_bursts.remove(entry)
            except:
                pass
            continue
        alpha = int(200 * (1.0 - age))
        radius = int(18 + age * 96)
        s = pygame.Surface((radius * 2, radius * 2), pygame.SRCALPHA)
        c = PLAYER_COLORS[player]
        pygame.draw.circle(s, (c.r, c.g, c.b), (radius, radius), radius, width=6)
        s.set_alpha(alpha)
        surf.blit(
            s,
            (pos[0] - radius, pos[1] - radius),
            special_flags=pygame.BLEND_PREMULTIPLIED,
        )


# ----------------------------
# Player icons drawing (animated) - positioned away from game pads
# ----------------------------
EDGE_ICON_SIZE = 144  # Increased by 20%


def draw_player_icons(surf):
    for p in range(3):
        if p == 0:  # Left side - move further from pads
            cx, cy = (
                PAD_MARGIN + EDGE_ICON_SIZE // 2 - 30,
                SCREEN_H // 2 - 200,
            )  # Move up and left
        elif p == 1:  # Bottom - move further from pads
            cx, cy = (
                SCREEN_W // 2,
                SCREEN_H - PAD_MARGIN - EDGE_ICON_SIZE // 2 - 50,
            )  # Move down more
        else:  # Right side - move further from pads
            cx, cy = (
                SCREEN_W - PAD_MARGIN - EDGE_ICON_SIZE // 2 + 30,
                SCREEN_H // 2 - 200,
            )  # Move up and right

        # Draw background circle
        box = pygame.Surface((EDGE_ICON_SIZE, EDGE_ICON_SIZE), pygame.SRCALPHA)
        pygame.draw.ellipse(
            box, (255, 255, 255, 24), (0, 0, EDGE_ICON_SIZE, EDGE_ICON_SIZE)
        )
        surf.blit(box, (cx - EDGE_ICON_SIZE // 2, cy - EDGE_ICON_SIZE // 2))

        anim = player_anim[p]["scale"]

        # Draw player image or fallback
        if p < len(player_images) and player_images[p]:
            try:
                img = pygame.transform.smoothscale(
                    player_images[p],
                    (int(EDGE_ICON_SIZE * anim), int(EDGE_ICON_SIZE * anim)),
                )
                surf.blit(img, (cx - img.get_width() // 2, cy - img.get_height() // 2))
            except Exception:
                draw_fallback_player_icon(surf, cx, cy, anim)
        else:
            draw_fallback_player_icon(surf, cx, cy, anim)


def draw_fallback_player_icon(surf, cx, cy, anim):
    """Draw fallback player icon when image is unavailable"""
    r = int(
        (EDGE_ICON_SIZE // 2 - 10) * anim
    )  # Slightly smaller to account for larger size
    pygame.draw.circle(surf, (255, 255, 255), (cx, cy), r)
    pygame.draw.circle(surf, (60, 60, 60), (cx - 22, cy - 14), max(5, int(12 * anim)))
    pygame.draw.circle(surf, (60, 60, 60), (cx + 22, cy - 14), max(5, int(12 * anim)))
    pygame.draw.arc(
        surf,
        (255, 120, 120),
        (cx - 43, cy - 12, 86, 72),  # Scaled arc
        math.pi / 8,
        math.pi - math.pi / 8,
        max(2, int(7 * anim)),
    )


# ----------------------------
# Edge pad drawing
# ----------------------------
def draw_edge_pads(surf):
    """Draw edge pads with better error handling"""
    try:
        for idx in range(TOTAL_BUTTONS):
            pad_pos = get_button_pos(idx)
            if pad_pos:
                px, py = pad_pos
                color = PAD_COLORS[idx % len(PAD_COLORS)]

                # Draw small colored circle - scaled up
                small_circle_x = px + PAD_RADIUS - 22
                small_circle_y = py - PAD_RADIUS + 22

                # Bounds check the circle position
                if 0 <= small_circle_x < SCREEN_W and 0 <= small_circle_y < SCREEN_H:
                    pygame.draw.circle(
                        surf, color, (small_circle_x, small_circle_y), 10
                    )

                    # Draw pad number for debugging
                    if DEBUG_PRINT_JOY:
                        font = pygame.font.Font(None, 24)
                        text = font.render(str(idx), True, (255, 255, 255))
                        text_rect = text.get_rect(center=(px, py))
                        surf.blit(text, text_rect)
    except Exception as e:
        print(f"Error drawing edge pads: {e}")


# ----------------------------
# Center hoop drawing - scaled up
# ----------------------------
def draw_center_hoop(surf):
    cx, cy = CENTER_POS
    if center_hoop_image:
        try:
            w = int(CENTER_RIM_RADIUS * 2.6 * 1.2)  # 20% larger
            h = int(
                w * (center_hoop_image.get_height() / center_hoop_image.get_width())
            )
            hi = pygame.transform.smoothscale(center_hoop_image, (w, h))
            surf.blit(hi, (cx - w // 2, cy - h // 2 - 7))  # Adjusted offset
            return
        except Exception:
            pass
    bw, bh = 288, 67  # 20% larger backboard
    board = pygame.Surface((bw, bh), pygame.SRCALPHA)
    pygame.draw.rect(board, (255, 255, 255, 230), (0, 0, bw, bh), border_radius=7)
    surf.blit(board, (cx - bw // 2, cy - bh // 2 - 29))
    rim_r = CENTER_RIM_RADIUS
    rim_surf = pygame.Surface((rim_r * 2, rim_r * 2), pygame.SRCALPHA)
    pygame.draw.circle(
        rim_surf, RIM_COLOR, (rim_r, rim_r), rim_r, width=10
    )  # Thicker rim
    surf.blit(rim_surf, (cx - rim_r, cy - rim_r // 2))


# ----------------------------
# Game state
# ----------------------------
balls = [
    ShootingBall() for _ in range(8)
]  # Fixed number of balls - moved after class definition
last_input_time = time.time()


def set_last_input_time():
    global last_input_time
    last_input_time = time.time()


# ----------------------------
# Input handling
# ----------------------------
def handle_touch(x, y):
    """Handle touch input with bounds checking - simplified version"""
    try:
        if not isinstance(x, (int, float)) or not isinstance(y, (int, float)):
            return

        x, y = int(x), int(y)
        if x < 0 or y < 0 or x >= SCREEN_W or y >= SCREEN_H:
            return

        set_last_input_time()
        print(f"Touch input at ({x}, {y})")

        # Check balls first
        for b in sorted(balls, key=lambda bl: -bl.bounce_phase):
            if b.state == "idle" and b.current_pos:
                try:
                    bx, by = int(b.current_pos[0]), int(b.current_pos[1])
                    r = b.radius + 18
                    if (x - bx) ** 2 + (y - by) ** 2 <= r * r:
                        print(f"  -> Touched ball at pad {b.pad_idx}")
                        b.shoot_to_center()
                        return
                except (TypeError, ValueError):
                    continue

        # Check pad positions
        for idx in range(TOTAL_BUTTONS):
            try:
                pad_pos = get_button_pos(idx)
                if pad_pos:
                    px, py = pad_pos
                    if (x - px) ** 2 + (y - py) ** 2 <= (PAD_RADIUS + 22) ** 2:
                        print(f"  -> Touched pad {idx}")
                        for b in balls:
                            if b.pad_idx == idx and b.state == "idle":
                                print(f"    -> Found ball, shooting!")
                                b.shoot_to_center()
                                return
                        print(f"    -> No ball at pad {idx}")
            except Exception:
                continue

        # Create touch feedback particles
        try:
            for _ in range(8):
                particles.append(
                    Particle(
                        (x + random.uniform(-8, 8), y + random.uniform(-8, 8)),
                        random.choice(PLAYER_COLORS),
                    )
                )
        except Exception:
            pass

        if SND_SHOOT:
            try:
                SND_SHOOT.play()
            except pygame.error:
                pass

    except Exception as e:
        print(f"Error in handle_touch: {e}")


# Track button states for shutdown confirmation
button_states = {}

# ----------------------------
# Main loop with better error handling
# ----------------------------
running = True
try:
    while running:
        try:
            dt = clock.get_time() / 1000.0
            if dt <= 0 or dt > 1.0:  # Cap extremely large dt values
                dt = 1.0 / FPS

            # Periodic asset reloading
            current_time = time.time()
            if current_time - last_asset_reload >= ASSET_RELOAD_INTERVAL:
                try:
                    reload_all_assets()
                    for b in balls:
                        if hasattr(b, "update_ball_image"):
                            b.update_ball_image()
                    last_asset_reload = current_time
                except Exception as e:
                    print(f"Asset reload failed: {e}")

            # Ensure balls list is valid and not empty
            if not balls or not isinstance(balls, list):
                balls = [ShootingBall() for _ in range(8)]
            elif len(balls) != 8:
                while len(balls) < 8:
                    balls.append(ShootingBall())
                while len(balls) > 8:
                    balls.pop()

            # Safe active pads calculation
            try:
                active_pads = {
                    b.pad_idx
                    for b in balls
                    if (
                        hasattr(b, "state")
                        and hasattr(b, "pad_idx")
                        and b.state in {"idle", "spawning"}
                        and b.pad_idx is not None
                    )
                }
            except Exception:
                active_pads = set()

            # Event handling with comprehensive error protection
            for ev in pygame.event.get():
                try:
                    if ev.type == pygame.QUIT:
                        running = False

                    elif ev.type == pygame.KEYDOWN:
                        if ev.key == pygame.K_ESCAPE:
                            running = False
                        elif ev.key in KEYBOARD_TO_BUTTON:
                            btn = KEYBOARD_TO_BUTTON[ev.key]
                            set_last_input_time()
                            button_states[f"kbd_{btn}"] = True
                            print(
                                f"Keyboard button {btn} pressed (key: {pygame.key.name(ev.key)})"
                            )
                            # Handle keyboard input for game buttons
                            for b in balls:
                                if (
                                    hasattr(b, "pad_idx")
                                    and hasattr(b, "state")
                                    and b.pad_idx == btn
                                    and b.state == "idle"
                                ):
                                    print(f"  -> Shooting ball at pad {btn}")
                                    b.shoot_to_center()
                                    break

                    elif ev.type == pygame.KEYUP:
                        if ev.key in KEYBOARD_TO_BUTTON:
                            btn = KEYBOARD_TO_BUTTON[ev.key]
                            button_states[f"kbd_{btn}"] = False

                    elif ev.type == pygame.JOYBUTTONDOWN:
                        btn = ev.button
                        joy_id = ev.joy

                        print(f"=== JOYSTICK BUTTON PRESS ===")
                        print(f"Button {btn} pressed on Joystick {joy_id}")
                        print(f"Game joystick ID: {game_joystick_id}")

                        # Handle ALL buttons from the game controller (simplified logic)
                        if joy_id == game_joystick_id:
                            set_last_input_time()
                            button_states[f"joy_{joy_id}_{btn}"] = True

                            # Check if button is in valid range
                            if 0 <= btn < TOTAL_BUTTONS:
                                print(f"  -> Valid game button {btn}")
                                # Find ball at this pad and shoot it
                                ball_found = False
                                for b in balls:
                                    if (
                                        hasattr(b, "pad_idx")
                                        and hasattr(b, "state")
                                        and b.pad_idx == btn
                                        and b.state == "idle"
                                    ):
                                        print(
                                            f"  -> Found ball at pad {btn}, shooting!"
                                        )
                                        b.shoot_to_center()
                                        ball_found = True
                                        break

                                if not ball_found:
                                    print(f"  -> No active ball at pad {btn}")
                                    # Create some visual feedback even if no ball
                                    pad_pos = get_button_pos(btn)
                                    if pad_pos:
                                        px, py = pad_pos
                                        for _ in range(5):
                                            try:
                                                particles.append(
                                                    Particle(
                                                        (
                                                            px
                                                            + random.uniform(-15, 15),
                                                            py
                                                            + random.uniform(-15, 15),
                                                        ),
                                                        random.choice(PLAYER_COLORS),
                                                    )
                                                )
                                            except Exception:
                                                pass
                            else:
                                print(
                                    f"  -> Button {btn} is outside valid range (0-{TOTAL_BUTTONS-1})"
                                )
                        else:
                            print(
                                f"  -> Button from non-game joystick {joy_id} (ignoring)"
                            )

                    elif ev.type == pygame.JOYBUTTONUP:
                        btn = ev.button
                        joy_id = ev.joy

                        if joy_id == game_joystick_id and 0 <= btn < TOTAL_BUTTONS:
                            button_states[f"joy_{joy_id}_{btn}"] = False
                            print(f"Button {btn} released on game joystick {joy_id}")

                    elif ev.type == pygame.MOUSEBUTTONDOWN:
                        if ev.button == 1:  # Left click
                            mx, my = ev.pos
                            print(f"Mouse click at ({mx}, {my})")
                            handle_touch(mx, my)

                    elif ev.type == pygame.FINGERDOWN:
                        fx = int(ev.x * SCREEN_W)
                        fy = int(ev.y * SCREEN_H)
                        print(f"Touch at ({fx}, {fy})")
                        handle_touch(fx, fy)

                except Exception as e:
                    print(f"Error handling event {ev.type}: {e}")
                    continue

            # Update game objects with error protection
            try:
                for b in balls[:]:
                    if hasattr(b, "update"):
                        b.update(dt, active_pads)
            except Exception as e:
                print(f"Error updating balls: {e}")

            try:
                for p in particles[:]:
                    if hasattr(p, "update") and hasattr(p, "life"):
                        p.update(dt)
                        if p.life <= 0:
                            particles.remove(p)
            except Exception as e:
                print(f"Error updating particles: {e}")

            try:
                update_player_animations(dt)
            except Exception as e:
                print(f"Error updating animations: {e}")

            # DRAW with comprehensive error protection
            try:
                screen.fill(BG_COLOR)
                draw_edge_pads(screen)
                draw_center_hoop(screen)

                for b in balls:
                    if hasattr(b, "draw"):
                        b.draw(screen)

                for p in particles:
                    if hasattr(p, "draw"):
                        p.draw(screen)

                render_success_bursts(screen)
                draw_player_icons(screen)
                pygame.display.flip()
            except Exception as e:
                print(f"Error in drawing: {e}")

            clock.tick(FPS)

        except Exception as e:
            print(f"Error in main loop iteration: {e}")
            time.sleep(0.1)

except KeyboardInterrupt:
    print("Game interrupted by user")
except Exception as e:
    print(f"Fatal error in main loop: {e}")
    import traceback

    traceback.print_exc()
finally:
    print("Shutting down game...")
    try:
        pygame.mixer.quit()
        pygame.quit()
    except:
        pass
    sys.exit()
