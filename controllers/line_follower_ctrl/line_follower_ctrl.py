import math
from controller import Robot

# 1. Inisialisasi robot dan timestep
robot = Robot()
timestep = int(robot.getBasicTimeStep())

# 2. Inisialisasi motor kiri dan kanan (velocity control)
left_motor = robot.getDevice('left_motor')
right_motor = robot.getDevice('right_motor')
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

# 3. Inisialisasi sensor garis inframerah dan kompas untuk presisi sudut 90 derajat
ds_left = robot.getDevice('ds_left')
ds_right = robot.getDevice('ds_right')
ds_left.enable(timestep)
ds_right.enable(timestep)

compass = robot.getDevice('compass')
compass.enable(timestep)

# 4. Parameter kecepatan dan kendali halus (smooth)
BASE_SPEED = -2.8       # Kecepatan maju di jalur lurus (negatif = maju)
MAX_TURN_SPEED = 3.2    # Kecepatan putar maksimum saat mulai belok
MIN_TURN_SPEED = 0.6    # Kecepatan putar minimum saat mendekati 90 derajat (biar smooth)
CENTER_CORR = 0.8       # Koreksi halus agar robot tetap di tengah garis
CORNER_ADVANCE = 14     # Langkah maju sebentar saat sensor kena tikungan agar poros roda pas di tengah garis baru

def get_heading():
    """Membaca sudut hadap (yaw) robot dalam radian dari Compass."""
    north = compass.getValues()
    return math.atan2(north[0], north[1])

def angle_diff(target, current):
    """Menghitung selisih sudut terpendek dalam rentang [-pi, pi]."""
    diff = target - current
    while diff > math.pi:
        diff -= 2.0 * math.pi
    while diff < -math.pi:
        diff += 2.0 * math.pi
    return diff

# 5. Kalibrasi otomatis nilai lantai putih (10 step awal)
left_sum = 0.0
right_sum = 0.0
samples = 0
for step in range(10):
    if robot.step(timestep) == -1:
        break
    if step >= 5:
        left_sum += ds_left.getValue()
        right_sum += ds_right.getValue()
        samples += 1

left_thresh = (left_sum / max(samples, 1)) - 5.0
right_thresh = (right_sum / max(samples, 1)) - 5.0

# Kunci arah awal (0, 90, 180, 270 derajat relatif terhadap posisi start)
base_heading = get_heading()
turn_count = 0

state = "FOLLOW"        # Status: FOLLOW -> ADVANCE -> TURN
advance_timer = 0
cooldown = 0
corner_streak = 0

# 6. Loop utama navigasi Line Follower
while robot.step(timestep) != -1:
    left_val = ds_left.getValue()
    right_val = ds_right.getValue()
    heading = get_heading()

    left_on_line = left_val < left_thresh
    right_on_line = right_val < right_thresh

    if state == "FOLLOW":
        if cooldown > 0:
            cooldown -= 1
            corner_streak = 0
        elif left_on_line and right_on_line:
            corner_streak += 1
        else:
            corner_streak = 0

        # Deteksi tikungan 90 derajat
        if cooldown == 0 and corner_streak >= 2:
            state = "ADVANCE"
            advance_timer = CORNER_ADVANCE
            corner_streak = 0
            left_speed = BASE_SPEED * 0.7
            right_speed = BASE_SPEED * 0.7
        else:
            # Kendali halus mempertahankan robot di tengah garis lurus
            target_h = base_heading + turn_count * (math.pi / 2.0)
            err_h = angle_diff(target_h, heading)
            steer = max(-0.8, min(0.8, err_h * 2.5))

            if left_on_line and not right_on_line:
                steer += CENTER_CORR
            elif right_on_line and not left_on_line:
                steer -= CENTER_CORR

            left_speed = BASE_SPEED + steer
            right_speed = BASE_SPEED - steer

    elif state == "ADVANCE":
        # Maju sedikit agar titik tengah poros roda berada tepat di atas garis baru
        advance_timer -= 1
        left_speed = BASE_SPEED * 0.6
        right_speed = BASE_SPEED * 0.6
        if advance_timer <= 0:
            turn_count = (turn_count + 1) % 4
            state = "TURN"

    elif state == "TURN":
        # Putar kiri halus (proporsional) menuju sudut tepat kelipatan 90 derajat
        target_h = base_heading + turn_count * (math.pi / 2.0)
        err = angle_diff(target_h, heading)

        if abs(err) < 0.03:  # Toleransi < 1.7 derajat -> tepat lurus di jalur baru
            state = "FOLLOW"
            cooldown = 35
            left_speed = BASE_SPEED
            right_speed = BASE_SPEED
        else:
            # Kecepatan putar mengecil otomatis mendekati 90 derajat agar smooth
            omega = max(MIN_TURN_SPEED, min(MAX_TURN_SPEED, abs(err) * 3.5))
            if err > 0:
                left_speed = omega
                right_speed = -omega
            else:
                left_speed = -omega
                right_speed = omega

    left_motor.setVelocity(left_speed)
    right_motor.setVelocity(right_speed)