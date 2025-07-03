from controller import Robot

def check_color_in_regions(camera, img, width, height, target):
    """
    Check for color in different horizontal regions of the image.
    Returns True if the target color is found in any region.
    """
    # Define color ranges
    color_ranges = {
        "red": {
            "r_min": 180, "r_max": 255,
            "g_min": 0, "g_max": 60,
            "b_min": 0, "b_max": 60
        },
        "yellow": {
            "r_min": 180, "r_max": 255,
            "g_min": 180, "g_max": 255,
            "b_min": 0, "b_max": 60
        },
        "pink": {
            "r_min": 180, "r_max": 255,
            "g_min": 0, "g_max": 60,
            "b_min": 180, "b_max": 255
        },
        "brown": {
            "r_min": 35, "r_max": 60,
            "g_min": 25, "g_max": 45,
            "b_min": 10, "b_max": 35
        },
        "green": {
            "r_min": 0, "r_max": 60,
            "g_min": 60, "g_max": 255,
            "b_min": 0, "b_max": 60
        }
    }
    
    # Check three vertical regions: left, center, and right
    regions_x = [width // 4, width // 2, 3 * width // 4]
    center_y = height // 2
    
    for x in regions_x:
        # Get RGB values for this point
        r = camera.imageGetRed(img, width, x, center_y)
        g = camera.imageGetGreen(img, width, x, center_y)
        b = camera.imageGetBlue(img, width, x, center_y)
        
        # Check if these values match the target color
        color = color_ranges[target]
        if (color["r_min"] <= r <= color["r_max"] and
            color["g_min"] <= g <= color["g_max"] and
            color["b_min"] <= b <= color["b_max"]):
            print(f"Color {target} detected at x={x}: R={r:.1f}, G={g:.1f}, B={b:.1f}")
            return True
            
    return False

def run_robot(robot):
    """Wall-following robot with precise color detection."""
    timestep = int(robot.getBasicTimeStep())
    max_speed = 6.28

    # Enable motors
    left_motor = robot.getDevice('left wheel motor')
    right_motor = robot.getDevice('right wheel motor')
    left_motor.setPosition(float('inf'))
    left_motor.setVelocity(0.0)
    right_motor.setPosition(float('inf'))
    right_motor.setVelocity(0.0)

    # Enable camera
    camera = robot.getDevice('camera')
    camera.enable(timestep)

    # Enable proximity sensors
    prox_sensors = []
    for ind in range(8):
        sensor_name = 'ps' + str(ind)
        prox_sensors.append(robot.getDevice(sensor_name))
        prox_sensors[ind].enable(timestep)

    # Color sequence
    color_sequence = ["red", "yellow", "pink", "brown", "green"]
    current_color_index = 0
    
    # States for final movement sequence
    WALL_FOLLOWING = 0
    FIRST_WALL_REACHED = 1
    TURNING_RIGHT = 2
    FINAL_APPROACH = 3
    STOPPED = 4
    
    final_state = WALL_FOLLOWING
    turn_timer = 0
    
    # Color detection parameters
    detection_count = 0  # Counter for consecutive detections
    REQUIRED_DETECTIONS = 3  # Number of consecutive detections required

    while robot.step(timestep) != -1:
        # Read proximity sensors
        sensor_values = [prox_sensors[ind].getValue() for ind in range(8)]
        front_wall = sensor_values[7] > 80
        left_wall = sensor_values[5] > 80
        left_corner = sensor_values[6] > 80

        # Print sensor values
        print(f"Sensor values: {[f'{val:.1f}' for val in sensor_values]}")

        # If all colors detected, execute final movement sequence
        if current_color_index >= len(color_sequence):
            if final_state == WALL_FOLLOWING:
                print("Following wall to first corner...")
                if front_wall:
                    final_state = FIRST_WALL_REACHED
                if left_wall:
                    print("Following the left wall.")
                    left_motor.setVelocity(max_speed)
                    right_motor.setVelocity(max_speed)
                else:
                    print("No left wall, turning left.")
                    left_motor.setVelocity(max_speed / 8)
                    right_motor.setVelocity(max_speed)

                if left_corner:
                    print("Too close to left corner, adjusting right.")
                    left_motor.setVelocity(max_speed)
                    right_motor.setVelocity(max_speed / 8)
                
            elif final_state == FIRST_WALL_REACHED:
                print("First wall reached, preparing to turn...")
                final_state = TURNING_RIGHT
                turn_timer = 0
                
            elif final_state == TURNING_RIGHT:
                print("Turning right...")
                left_motor.setVelocity(max_speed)
                right_motor.setVelocity(-max_speed)
                turn_timer += 1
                if turn_timer > 15:
                    final_state = FINAL_APPROACH
                    
            elif final_state == FINAL_APPROACH:
                print("Moving to final wall...")
                if front_wall:
                    final_state = STOPPED
                left_motor.setVelocity(max_speed)
                right_motor.setVelocity(max_speed)
                
            elif final_state == STOPPED:
                print("Final wall reached. Stopping.")
                left_motor.setVelocity(0)
                right_motor.setVelocity(0)
                break
                
            continue

        # Check camera for color detection
        if camera.getImage():
            img = camera.getImage()
            width = camera.getWidth()
            height = camera.getHeight()

            # Check for current target color
            if current_color_index < len(color_sequence):
                target_color = color_sequence[current_color_index]
                
                if check_color_in_regions(camera, img, width, height, target_color):
                    detection_count += 1
                    print(f"Potential {target_color} detection ({detection_count}/{REQUIRED_DETECTIONS})")
                    
                    if detection_count >= REQUIRED_DETECTIONS:
                        print(f"Confirmed {target_color.capitalize()}!")
                        current_color_index += 1
                        detection_count = 0
                        # Stop briefly after detecting color
                        left_motor.setVelocity(0)
                        right_motor.setVelocity(0)
                        robot.step(1000)  # Pause for 1 second
                else:
                    detection_count = 0
                    print(f"Searching for {target_color.capitalize()}...")

        # Wall-following logic during color detection
        if current_color_index < len(color_sequence):
            left_speed = max_speed
            right_speed = max_speed

            if front_wall:
                print("Front wall detected, turning right.")
                left_speed = max_speed
                right_speed = -max_speed
            elif left_wall:
                print("Following the left wall.")
                left_speed = max_speed
                right_speed = max_speed
            else:
                print("No left wall, turning left.")
                left_speed = max_speed / 8
                right_speed = max_speed

            if left_corner:
                print("Too close to left corner, adjusting right.")
                left_speed = max_speed
                right_speed = max_speed / 8

            # Set motor speeds
            left_motor.setVelocity(left_speed)
            right_motor.setVelocity(right_speed)

if __name__ == "__main__":
    my_robot = Robot()
    run_robot(my_robot)