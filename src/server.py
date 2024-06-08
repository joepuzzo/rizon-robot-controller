import socketio
import json
import time
from robot import Robot
from debug import Debug

logger = Debug('rizon:server\t')

sio = socketio.Client()


def emit(event, data):
    if sio.connected:
        sio.emit(event, data, namespace='/robot')
    else:
        logger(f'Socket not connected. Cannot emit {event}')


def start_server(config):
    # Create socket
    connection_string = f"http://{config['host']}:{config['port']}?id={config['id']}"

    if config.get('url', False):
        connection_string = f"{config['url']}?id={config['id']}"

    logger(f'Creating socket with connection string: {connection_string}')

    logger(f'Creating robot with config', json.dumps(config, indent=4))

    # Create robot
    robot = Robot(config)

    # ---------- Subscribe to socket events ----------
    @sio.event(namespace='/robot')
    def connect():
        logger('Connected to server')
        if robot.ready:
            # Need this sleep to ensure connection is good
            time.sleep(1)
            logger('Sending register and state')
            emit('register', robot.meta)
            emit('state', robot.state)

    @sio.event(namespace='/robot')
    def connect_error(data):
        logger(f'Failed to connect to server: {data}')

    @sio.event(namespace='/robot')
    def disconnect():
        logger('Disconnected from server')

    # Attempt to connect to the server
    logger('Attempting to connect to the server...')
    sio.connect(connection_string, namespaces=['/robot'])

    # ---------- Subscribe to robot events ----------
    @robot.on('state')
    def on_state():
        logger('Sending state')
        emit('state', robot.state)

    @robot.on('encoder')
    def on_encoder():
        # Specifically don't log here (too much logging)
        emit('encoder', robot.state)

    @robot.on('ready')
    def on_ready():
        logger('Robot ready, sending state and registering')
        emit('register', robot.meta)
        emit('state', robot.state)

    @robot.on('meta')
    def on_meta():
        logger('Sending meta')
        emit('register', robot.meta)

    @robot.on('moved')
    def on_moved():
        logger('Sending moved')
        emit('moved', robot.meta)

    @robot.on('grasped')
    def on_grasped():
        logger('Sending grasped')
        emit('grasped', robot.meta)

    @robot.on('zeroedFT')
    def on_zeroed_ft():
        logger('Sending zeroedFT')
        emit('zeroedFT', robot.meta)

    @robot.on('actionsComplete')
    def on_actions_complete(name):
        logger('Sending actionsComplete')
        emit('actionsComplete', name)

    @robot.on('pulse')
    def on_pulse(id, pos):
        emit('pulse', id)

    # ---------- Subscribe to controller commands ----------
    @sio.on('hello', namespace='/robot')
    def on_hello(msg):
        logger('Controller says hello')

    # ---------- MOTOR COMMANDS ----------

    @sio.on('motorSetPos', namespace='/robot')
    def on_motor_set_pos(id, pos, speed=0.1):
        logger(
            f"Controller says setMotorPos to {pos} at speed {speed} for motor {id}")
        robot.motor_set_position(id, pos, speed)

    @sio.on('motorHome', namespace='/robot')
    def on_motor_home(id):
        logger(f"Controller says motorHome {id}")
        robot.motor_home(id)

    @sio.on('motorResetErrors', namespace='/robot')
    def on_motor_reset_errors(id):
        logger(f"NOT SUPPORTED: motorResetErrors is not supported on Rizon Robot")

    @sio.on('motorEnable', namespace='/robot')
    def on_motor_enable(id):
        logger(f"NOT SUPPORTED: motorEnable is not supported on Rizon Robot")

    @sio.on('motorDisable', namespace='/robot')
    def on_motor_disable(id):
        logger(f"NOT SUPPORTED: motorDisable is not supported on Rizon Robot")

    @sio.on('motorCalibrate', namespace='/robot')
    def on_motor_calibrate(id):
        logger(f"NOT SUPPORTED: motorCalibrate is not supported on Rizon Robot")

    @sio.on('motorReference', namespace='/robot')
    def on_motor_reference(id):
        logger(f"NOT SUPPORTED: motorReference is not supported on Rizon Robot")

    @sio.on('motorZero', namespace='/robot')
    def on_motor_zero(id):
        logger(f"NOT SUPPORTED: motorZero is not supported on Rizon Robot")

    # ---------- GRIPPER COMMANDS ----------

    @sio.on('gripperSetPos', namespace='/robot')
    def on_gripper_set_pos(pos, speed, force, wait):
        logger(
            f"Controller says gripperSetPos to {pos}% at speed {speed} and a force of {force} and will wait {wait}")
        robot.gripper_set_position(pos, speed, force, wait)

    # ---------- ROBOT COMMANDS ----------

    @sio.on('robotStop', namespace='/robot')
    def on_robot_stop():
        logger(f"Controller says stop robot")
        robot.robot_stop()

    @sio.on('robotMode', namespace='/robot')
    def on_robot_mode(mode):
        logger(f"Controller says put the robot into {mode}")
        if mode == "IDLE":
            robot.robot_idle()

        logger('Sending mode change event')
        emit('mode', mode)

    @sio.on('robotFreeze', namespace='/robot')
    def on_robot_freeze():
        logger(f"Controller says freeze robot")
        robot.robot_freeze()

    @sio.on('robotEnable', namespace='/robot')
    def on_robot_enable():
        logger(f"Controller says enable robot")
        robot.robot_enable()

    @sio.on('robotCenter', namespace='/robot')
    def on_robot_center():
        logger(f"Controller says center robot")
        robot.robot_center()

    @sio.on('robotRunPlan', namespace='/robot')
    def on_robot_run_plan(name):
        logger(f"Controller says run plan {name}")
        robot.run_plan(name)

    @sio.on('robotSetAngles', namespace='/robot')
    def on_robot_set_angles(angles, speed=None, idle=None):

        if speed is None:
            speed = 0.1

        if idle is None:
            idle = True

        logger(
            f"Controller says setAngles for robot with angles {angles}, speed {speed} and idle {idle}")
        robot.robot_set_angles(angles, speed, idle)

    @sio.on('robotMoveL', namespace='/robot')
    def on_robot_MoveL(parameters):
        logger(
            f"Controller says robotMoveL with parameters {json.dumps(parameters,  indent=4)}"
        )

        # Get parameters off of the event
        position = parameters["position"]
        frame = parameters["frame"]
        speed = parameters["speed"]
        preferJntPos = parameters["preferJntPos"]
        idle = parameters.get("idle", True)

        # Call the robots moveL command
        target = ' '.join(map(str, position))
        jointString = ' '.join(map(str, preferJntPos))
        robot.move_l(target=target, frame=frame,
                     maxVel=speed, preferJntPos=jointString, stop=idle)

    @sio.on('robotMoveContact', namespace='/robot')
    def on_robot_MoveContact(parameters):
        logger(
            f"Controller says robotMoveContact with parameters {json.dumps(parameters,  indent=4)}"
        )

        # Get parameters off of the event
        contactDir = parameters["contactDir"]
        contactVel = parameters["contactVel"]
        maxContactForce = parameters["maxContactForce"]
        idle = parameters.get("idle", True)

        # Call the robots moveL command
        contactDirStr = ' '.join(map(str, contactDir))
        robot.move_contact(contactDir=contactDirStr, contactVel=contactVel,
                           maxContactForce=maxContactForce, stop=idle)

    @sio.on('robotRunActions', namespace='/robot')
    def on_robot_run_actions(actions):
        logger(f"Controller says robotRunActions")
        robot.run_actions(actions)

    @sio.on('robotUpdateConfig', namespace='/robot')
    def on_robot_update_config(key, value):
        logger(f"Controller says updateConfig for robot")
        robot.update_config(key, value)

    @sio.on('robotWriteConfig', namespace='/robot')
    def on_robot_write_config():
        logger(f"Controller says writeConfig for robot")
        robot.write_config()

    @sio.on('robotAccelEnabled', namespace='/robot')
    def on_robot_accel_enabled(value):
        logger(f"Controller says accelEnabled for robot to {value}")
        robot.robot_accel_enabled(value)

    @sio.on('robotResetErrors', namespace='/robot')
    def on_robot_reset_errors():
        logger(f"Controller says resetErrors for robot")
        robot.robot_reset()

    @sio.on('robotFreedriveEnable', namespace='/robot')
    def on_robot_freedrive_enable(frame, cartFloatingAxis, nullspace=False):
        logger(
            f"Controller says enable freedrive with frame {frame}, cartFloatingAxis {json.dumps(cartFloatingAxis,  indent=4)}, and nullspace {nullspace}"
        )
        robot.robot_freedrive_enable(frame, cartFloatingAxis, nullspace)

    @sio.on('robotJointFreedriveEnable', namespace='/robot')
    def on_robot_joint_freedrive_enable(joints):
        logger(
            f"Controller says enable joint freedrive with joints {json.dumps(joints, indent=4)}"
        )
        robot.robot_joint_freedrive_enable(joints)

    @sio.on('robotFreedriveDisable', namespace='/robot')
    def on_robot_freedrive_disable():
        logger(f"Controller says disable freedrive")
        robot.robot_freedrive_disable()

    @sio.on('robotHome', namespace='/robot')
    def on_robot_home():
        logger(f"NOT SUPPORTED: robotHome is not supported on Rizon Robot")

    @sio.on('robotSplitHome', namespace='/robot')
    def on_robot_split_home():
        logger(f"NOT SUPPORTED: robotSplitHome is not supported on Rizon Robot")

    @sio.on('robotReference', namespace='/robot')
    def on_robot_reference():
        logger(f"NOT SUPPORTED: robotReference is not supported on Rizon Robot")

    @sio.on('robotZero', namespace='/robot')
    def on_robot_zero():
        logger(f"NOT SUPPORTED: robotZero is not supported on Rizon Robot")

    @sio.on('robotZeroFT', namespace='/robot')
    def on_robot_zero_ft():
        logger(f"Controller says zero force tourque robot")
        robot.zero_ft_sensors()

    sio.wait()
