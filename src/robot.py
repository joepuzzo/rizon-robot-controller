import os
import json
import time
import threading
from pathlib import Path
from pyee import EventEmitter
from motor import Motor
from debug import Debug
from utility import toEuler
from utility import list2str
from utility import parse_pt_states
import math

# Import Flexiv RDK Python library
# fmt: off
import sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "lib"))
import flexivrdk
# fmt: on

logger = Debug('rizon:robot\t')


class Robot(EventEmitter):
    def __init__(self, config):
        logger(
            f'Creating robot with id {config["id"]}, robot_ip {config["robot_ip"]}, and local_ip {config["local_ip"]}')

        # Because we are event emitter
        super().__init__()

        # Setup robot properties
        self.id = config['id']  # id of the robot
        self.stopped = True  # if the robot is currently stopped ( disabled )
        self.ready = False  # if the robot is ready
        self.homing = False  # if the robot is currently homing
        self.home = False  # if the robot is home
        self.moving = False  # if the robot is moving
        self.freedrive = False  # if the robot is currently in freedrive
        self.errors = []  # array of any errors that got triggered

        # Setup rdk properties
        self.robot_ip = config['robot_ip']
        self.local_ip = config['local_ip']

        # Reference to the robots modes
        self.mode = flexivrdk.Mode

        # Initialize robot
        self.setup()

    def _initialize_robot(self):
        # Logic to initialize and return robot connection
        self.robot = flexivrdk.Robot(self.robot_ip, self.local_ip)

        # Handling faults if any
        self.robot_reset()

    def setup(self):
        # First read in the config
        self.read_config()

        logger(f'Starting robot with id {self.id}')

        # Call internal initialize function to set up rdk robot instance
        self._initialize_robot()

        # Instantiate gripper control interface
        self.gripper = flexivrdk.Gripper(self.robot)

        # RDK Robot state object
        self.robotState = flexivrdk.RobotStates()

        # Report robot updates updates at 100ms interval
        self.robot_updates = threading.Timer(0.1, self.robot_encoder)
        self.robot_updates.start()

        # We are now ready
        self.ready = True
        self.emit('ready')

    def read_config(self):
        # Read in config file (create if it does not exist yet)
        try:
            # Get filename
            filename = Path('config.json')

            # Check if it exists and create if it does not
            if not filename.exists():
                logger('Config file does not exist, creating')
                with open(filename, 'w') as f:
                    json.dump({
                        "j0": {"limNeg": -160, "limPos": 160, "homePos": 0},
                        "j1": {"limNeg": -130, "limPos": 130, "homePos": 0},
                        "j2": {"limNeg": -170, "limPos": 170, "homePos": 0},
                        "j3": {"limNeg": -107, "limPos": 154, "homePos": 0},
                        "j4": {"limNeg": -170, "limPos": 170, "homePos": 0},
                        "j5": {"limNeg": -80, "limPos": 260, "homePos": 0},
                        "j6": {"limNeg": -170, "limPos": 170, "homePos": 0}
                    }, f)

            # Read in config file
            with open(filename, 'r') as f:
                self.config = json.load(f)

            logger('Successfully read in config',
                   json.dumps(self.config, indent=4))
        except Exception as e:
            logger(f'Error reading config: {e}')

    def write_config(self):
        logger('Writing config to file', self.config)
        try:
            # Get filename
            filename = Path('config.json')
            # Write config
            with open(filename, 'w') as f:
                json.dump(self.config, f)
        except Exception as e:
            logger(f'Error writing config: {e}')

    def update_config(self, key, value, save=False):
        logger(f'Updating config {key} to {value}')

        # Special check (don't let user set a config param to null!!)
        if value is None:
            logger(f'Unable to set {key} to {value} as it is null')
            return

        # Example key = "j0.limitAdj"
        if '.' in key:
            joint, param = key.split('.')
            # Update the config
            self.config[joint][param] = value
            # Update the motor
            self.motor_map[joint][param] = value
        else:
            self.config[key] = value

        # Now write the config out
        if save:
            self.write_config()

        logger('Updated config', self.config)
        self.emit('meta')

    @property
    def state(self):

        # Update the state variable
        self.robot.getRobotStates(self.robotState)

        # Build motors state object from self.robotState.q
        motors = {}
        for i, angle in enumerate(self.robotState.q):
            motors[f'j{i}'] = {
                'angle': math.degrees(angle)
            }

        return {
            'id': self.id,
            'motors': motors,
            'tcpPose': toEuler(self.robotState.tcpPose),
            'flangePose': toEuler(self.robotState.flangePose),
            'joints': [math.degrees(angle) for angle in self.robotState.q],
            'extWrenchInTcp': self.robotState.extWrenchInTcp
        }

    @property
    def meta(self):
        # Build motors meta object
        motors = {f'j{i}': {'id': f'j{i}'} for i in range(7)}

        return {
            'stopped': self.stopped,
            'ready': self.ready,
            'home': self.home,
            'homing': self.homing,
            'moving': self.moving,
            'busy': self.busy,
            'config': self.config,
            'motors': motors,
            'errors': self.errors,
            'freedrive': self.freedrive
        }

    @property
    def busy(self):
        return self.robot.isBusy()

    def validate(self, enabled=False, cleared=False, moving=False, log=''):
        # If action requires robot to be enabled and we are not then error out
        if enabled and self.stopped:
            message = f'Enable before {log}'
            logger(message)
            self.errors.append({'type': 'warning', 'message': message})
            self.emit('meta')
            return False
        # If action requires robot to have zero errors and we have errors then error out
        if cleared and self.errors:
            message = f'Clear errors before {log}'
            logger(message)
            self.errors.append({'type': 'warning', 'message': message})
            self.emit('meta')
            return False
        # If action requires robot to be NOT moving
        if moving and self.moving:
            message = f'Robot is currently moving, unable to {log}'
            logger(message)
            self.errors.append({'type': 'warning', 'message': message})
            self.emit('meta')
            return False

        return True

    def robot_state(self):
        self.emit('state')

    def robot_encoder(self):
        self.emit('encoder')
        self.encoder_timer = threading.Timer(0.1, self.robot_encoder)
        self.encoder_timer.start()

    def robot_stop(self):
        logger('Stop robot')
        self.robot.stop()
        self.freedrive = False
        self.stopped = True
        self.emit('meta')

    def robot_freeze(self):
        logger('Freeze robot')
        self.robot.stop()
        self.freedrive = False
        # note specifically dont set stopped as we are still enabled
        self.emit('meta')

    def robot_center(self):
        logger('Center robot')
        self.robot_set_angles([0, 0, 0, 0, 0, 0, 0])
        self.emit('meta')

    def robot_enable(self):
        logger('Attemptig to Enable robot')

        try:
            # Clear any errors or faults
            self.robot_reset()

            # Enabling the robot
            logger("Enabling robot ...")
            self.robot.enable()

            # Wait for the robot to become operational
            seconds_waited = 0
            while not self.robot.isOperational():
                time.sleep(1)
                seconds_waited += 1
                if seconds_waited == 10:
                    logger(
                        "Still waiting for robot to become operational, please check that the robot 1) "
                        "has no fault, 2) is in [Auto (remote)] mode")
                    return

            # If we get here the Robot is now operational
            self.stopped = False
            logger("Robot is now operational")

        except Exception as e:
            # Print exception error message
            self.errors.append({'type': 'error', 'message': str(e)})
            logger(str(e))

        self.emit('meta')

    def robot_reset(self):
        logger('Clearing any robot errors/faults')

        try:
            # Check if the robot has fault
            if self.robot.isFault():
                logger("Fault occurred on robot server, trying to clear ...")
                # Try to clear the fault
                self.robot.clearFault()
                time.sleep(2)
                # Check again
                if self.robot.isFault():
                    logger("Fault cannot be cleared, exiting ...")
                    return
                logger("Fault on robot server is cleared")
            else:
                logger("No fault on robot server")

            # Reset all robot errors
            self.errors = []

        except Exception as e:
            # Print exception error message
            self.errors.append({'type': 'error', 'message': str(e)})
            logger(str(e))

        self.emit('meta')

    def robot_set_angles(self, angles, speed=0.1):
        logger(f'robotSetAngles at speed {speed} angles: {angles}')
        # Validate action
        if not self.validate(enabled=True, cleared=True, moving=True, log='move the robot'):
            return

        # Set all motors to a position via move_j
        target = ' '.join(map(str, angles))
        self.move_j(target, speed)

    # -------------------- Freedrive Functions --------------------

    def robot_freedrive_enable(self, frame, cartFloatingAxis, nullspace=False):
        logger('Enabling freedrive')

        # Validate action
        if not self.validate(enabled=True, cleared=True, moving=True, log='attempting to freedrive'):
            return

        x = cartFloatingAxis['x']
        y = cartFloatingAxis['y']
        z = cartFloatingAxis['z']
        rx = cartFloatingAxis['rx']
        ry = cartFloatingAxis['ry']
        rz = cartFloatingAxis['rz']

        if self.busy:
            logger("Cannot enable freedrive, robot is busy")
            return

        self.freedrive = True

        # Zero FT sensors
        self.zero_ft_sensors()

        self.freedrive = True

        self.emit('meta')

        # Switch to primitive execution mode for single-axis freedrive
        self.robot.setMode(self.mode.NRT_PRIMITIVE_EXECUTION)

        # Convert the boolean values to their integer counterparts (1 for True, 0 for False)
        floatingAxis = f"{int(x)} {int(y)} {int(z)} {int(rx)} {int(ry)} {int(rz)}"
        cartFloatingFrame = frame

        logger("Executing primitive: FloatingCartesian")

        enableJointFloating = 1 if nullspace else 0

        # Send command to robot
        self.robot.executePrimitive(
            f"FloatingCartesian(cartFloatingAxis={floatingAxis}, cartFloatingFrame={cartFloatingFrame}, enableJointFloating={enableJointFloating})"
        )

        self.emit('meta')

    def robot_joint_freedrive_enable(self, joints):
        logger(f'Enabling Joint Freedrive for joints {joints}')

        # Validate action
        if not self.validate(enabled=True, cleared=True, moving=True, log='attempting to freedrive joints'):
            return

        if self.busy:
            logger("Cannot enable freedrive, robot is busy")
            return

        try:

            j0 = joints.get('j0', False)
            j1 = joints.get('j1', False)
            j2 = joints.get('j2', False)
            j3 = joints.get('j3', False)
            j4 = joints.get('j4', False)
            j5 = joints.get('j5', False)
            j6 = joints.get('j6', False)

            self.freedrive = True

            # Zero FT sensors
            self.zero_ft_sensors()

            self.freedrive = True

            self.emit('meta')

            # Switch to primitive execution mode for single-axis freedrive
            self.robot.setMode(self.mode.NRT_PRIMITIVE_EXECUTION)

            # Convert the boolean values to their integer counterparts (1 for True, 0 for False)
            floatingJoints = f"{int(j0)} {int(j1)} {int(j2)} {int(j3)} {int(j4)} {int(j5)} {int(j6)}"

            logger("Executing primitive: floatingSoft")

            # Send command to robot
            self.robot.executePrimitive(
                f"floatingSoft(jointFloatingSwitch={floatingJoints})"
            )

        except Exception as e:
            # Print exception error message
            self.errors.append({'type': 'error', 'message': str(e)})
            logger(str(e))

        self.emit('meta')

    def robot_freedrive_disable(self):
        logger('Disabling freedrive')
        self.freedrive = False
        self.robot.stop()
        self.emit('meta')

    # -------------------- Motor Functions --------------------

    def motor_set_position(self, id, pos, speed=0.1):
        logger(f'Set position for motor {id} to pos {pos} at speed {speed}')

        # Validate action
        if not self.validate(enabled=True, cleared=True, moving=True, log=f'move motor {id}'):
            return

        # Update the state variable
        self.robot.getRobotStates(self.robotState)

        # Get current angles
        angles = [math.degrees(angle) for angle in self.robotState.q]

        logger(f'Current angles are {angles}')

        # Replace the angle at the position we want to set id = "j3" then id[1:] will = "3"
        angles[int(id[1:])] = pos

        # Update the position
        self.robot_set_angles(angles, speed)

    def motor_home(self, id):
        logger(f'Home motor {id}')
        self.motor_map[id].go_home()

    # -------------------- Gripper Functions --------------------

    def gripper_set_position(self, pos, speed=0.01, force=0):

        # Convert the % open 0-100 into a width in meters
        width = pos / 1000.0

        logger(
            f'Opening gripper to {pos}% ( width of {width}m ), at a speed of {speed}m/s, and a force of {force}N')

        try:

            # Set to primitive execution if we need to
            if self.robot.getMode() != self.mode.NRT_PRIMITIVE_EXECUTION:
                print(
                    f"Setting to {self.mode.NRT_PRIMITIVE_EXECUTION} before moveJ")
                self.robot.setMode(self.mode.NRT_PRIMITIVE_EXECUTION)

            # Move the gripper
            self.gripper.move(width, speed, force)

        except Exception as e:
            # Print exception error message
            self.errors.append({'type': 'error', 'message': str(e)})
            logger(str(e))

        self.emit('meta')

    # -------------------- Move Functions ---------------------

    def move_j(self, target, vel, stop=True):

        # Validate action
        if not self.validate(enabled=True, cleared=True, moving=True, log='moveJ'):
            return

        try:

            # Set to primitive execution if we need to
            if self.robot.getMode() != self.mode.NRT_PRIMITIVE_EXECUTION:
                logger(
                    f"Setting to {self.mode.NRT_PRIMITIVE_EXECUTION} before moveJ")
                self.robot.setMode(self.mode.NRT_PRIMITIVE_EXECUTION)

            # Build command
            command = f"MoveJ(target={target}, vel={vel})"
            logger(f"Executing Command: {command}")

            # We are moving to a new location
            self.moving = True
            self.emit('meta')

            # Execute the command
            self.robot.executePrimitive(command)

            self.emit('meta')

            # Wait for reached target
            while (parse_pt_states(self.robot.getPrimitiveStates(), "reachedTarget") != "1"):
                time.sleep(0.1)

            # We are no longer moving to a new location
            self.moving = False

            if stop:
                self.robot.stop()

        except Exception as e:
            # Print exception error message
            self.errors.append({'type': 'error', 'message': str(e)})
            logger(str(e))

        # Note order is important here, we need to make sure meta is sent before moved
        # This will ensure anyone listening to move event has the corect meta state of this robot :)
        self.emit('meta')
        self.emit('moved')

    def move_l(self, target, frame='WORLD WORLD_ORIGIN', maxVel="0.1", preferJntPos=None, stop=True):

        # Validate action
        if not self.validate(enabled=True, cleared=True, moving=True, log='moveL'):
            return

        try:
            # Set to primitive execution if we need to
            if self.robot.getMode() != self.mode.NRT_PRIMITIVE_EXECUTION:
                logger(
                    f"Setting to {self.mode.NRT_PRIMITIVE_EXECUTION} before moveL")
                self.robot.setMode(self.mode.NRT_PRIMITIVE_EXECUTION)

            # Build command
            command = f"MoveL(target={target} {frame}, maxVel={maxVel})"

            if preferJntPos:
                command = f"MoveL(target={target} {frame}, maxVel={maxVel}, preferJntPos={preferJntPos})"

            # We are moving to a new location
            self.moving = True
            self.emit('meta')

            # Execute the command
            self.robot.executePrimitive(command)

            self.emit('meta')

            # Wait for reached target
            while (parse_pt_states(self.robot.getPrimitiveStates(), "reachedTarget") != "1"):
                time.sleep(0.1)

            if stop:
                self.robot.stop()

            self.moving = False

        except Exception as e:
            # Print exception error message
            self.errors.append({'type': 'error', 'message': str(e)})
            logger(str(e))

        # Note order is important here, we need to make sure meta is sent before moved
        # This will ensure anyone listening to move event has the corect meta state of this robot :)
        self.emit('meta')
        self.emit('moved')

    # -------------------- Force Tourque ---------------------

    def zero_ft_sensors(self):

        # Get and print the current TCP force/moment readings
        robot_states = flexivrdk.RobotStates()
        self.robot.getRobotStates(robot_states)
        logger(
            "TCP force and moment reading in base frame BEFORE sensor zeroing: " +
            list2str(robot_states.extWrenchInBase) + "[N][Nm]")

        # Run the "ZeroFTSensor" primitive to automatically zero force and torque sensors
        self.robot.setMode(self.mode.NRT_PRIMITIVE_EXECUTION)
        self.robot.executePrimitive("ZeroFTSensor()")

        # WARNING: during the process, the robot must not contact anything, otherwise the result
        # will be inaccurate and affect following operations
        logger(
            "Zeroing force/torque sensors, make sure nothing is in contact with the robot")

        # Wait for the primitive completion
        while (self.robot.isBusy()):
            time.sleep(1)
        logger("Sensor zeroing complete")

        # Get and print the current TCP force/moment readings
        self.robot.getRobotStates(robot_states)
        logger(
            "TCP force and moment reading in base frame AFTER sensor zeroing: " +
            list2str(robot_states.extWrenchInBase) + "[N][Nm]")

        self.emit('meta')
