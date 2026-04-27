#UPDATED OCTOBER 1ST 2025 
#V1.2

import os
import serial
import threading
import time

class DeltaX():
    CW = 0
    CCW = 1
    OFF = 0
    ON = 65536
    
    #robot model
    DeltaX_S = 0
    DeltaX_V2 = 1

    #gcode - don't edit for now
    Gcode_None = 1
    Gcode_G_M = 0 
    Gcode_Macro = 2

    #axis
    AXIS_XYZ = 0
    AXIS_W = 1
    AXIS_U = 2
    AXIS_V = 3

    #parameter
    ROBOT_V = 0
    ROBOT_A = 1
    ROBOT_J = 2
    ROBOT_VS = 3
    ROBOT_VE = 4

    #
    ERROR = 0
    DONE = 1
    NO_REPLY = 2

    #end effector
    Vacuum = 0
    Gripper = 1
    Pen = 2
    Laser = 3
    Printer = 4
    Custom = 5

    
    def __init__(self, port = "None", baudrate = 115200, model = DeltaX_S):
        self.comport = port
        self.baudrate = baudrate
        self.model = model
        self.__serial = serial.Serial()
        self.__read_thread = None
        self.__is_connected = False
        self.__real_position = [0.0, 0.0, -750.0, 0.0, 0.0]
        self.__real_angle = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.__latest_response = ''
        self.__gcode_state = DeltaX.DONE
        self.__a_input = [0, 0, 0, 0]
        self.__i_input = [0, 0, 0, 0, 0, 0, 0, 0] 
        self.__feedback_queue = [] #SUS
        self.__parameter = [1000.0, 20000.0, 1000000.0, 20.0, 20.0]
        self.__w_parameter = [400.0, 8000.0, 1000000.0, 20.0, 20.0]
        self.__u_parameter = [400.0, 8000.0, 1000000.0, 20.0, 20.0]
        self.__v_parameter = [400.0, 8000.0, 1000000.0, 20.0, 20.0]
        self.timeout = 15 #find a way to not rely on this
        self.__connect_timeout = 3
        self.__is_connecting = False
        self.__last_time = time.time() #global time var

    def connect(self):
        """Open comport and connect with robot."""
        self.__serial.port = self.comport
        self.__serial.baudrate = self.baudrate
        self.__serial.timeout = 0


        try:
            self.__serial.open()
        except Exception as e:
            print(f"[Connect Error] Failed to open serial port: {e}")
            return False  # Don't proceed if port can't open


        if self.__serial.isOpen():
            print("[Connect] Serial port opened successfully.")
            self.__feedback_queue.clear()
            self.__send_gcode_to_robot('IsDelta')
            self.__last_time = time.time()
            self.__is_connecting = True


            self.__read_thread = threading.Thread(
                target=self.__serial_read_event,
                args=(self.__serial,),
                daemon=True
            )
            self.__read_thread.start()
            self.polling_wait()
            return self.__is_connected
        else:
            print("[Connect Error] Serial port failed to open.")
            return False

    def disconnect(self):
        """Disconnect with robot."""
        self.__is_connected = False
        
        try:
            self.__serial.close()
        except:           
            pass

    def is_connected(self):
        """Return is robot connected."""
        return self.__is_connected
        
    def __serial_read_event(self, ser):
        buffer = ""
        while ser.isOpen():
        # Connection timeout handling
            if self.__is_connecting:
                if time.time() - self.__last_time > self.__connect_timeout:
                    print("[Connection] Timeout reached. Disconnecting...")
                    self.disconnect()
                    self.__is_connecting = False
                    self.__feedback_queue.clear()
            else:
            # Feedback timeout handling
                if len(self.__feedback_queue) > 0:
                    if time.time() - self.__last_time > self.timeout:
                        print("[Feedback] No reply received. Clearing queue.")
                        self.__gcode_state = DeltaX.NO_REPLY
                        self.__feedback_queue.clear()
                else:
                    self.__last_time = time.time()


            time.sleep(0.002)


            try:
                chunk = ser.read(ser.in_waiting or 1).decode(errors='ignore')
                buffer += chunk
                if '\n' in buffer:
                    lines = buffer.split('\n')
                    for line in lines[:-1]:
                        response = line.strip()
                        if response:
                            print(f"[Serial] Received: {response}")
                            self.__last_time = time.time()
                            try:
                                self.__response_handling(response)
                            except Exception as e:
                                print(f"[Handler Error] Failed to process response: {e}")
                    buffer = lines[-1]  # Keep the last partial line
            except Exception as e:
                print(f"[Serial Error] Failed to read or decode: {e}")

    def __remote_feedback_queue(self, gcode_type):
        if not self.__feedback_queue:
            return
    # Handle Gcode_None: treat as a generic "pop" if queue is not empty
        if gcode_type == DeltaX.Gcode_None:
            removed = self.__feedback_queue.pop(0)
            #print(f"[FeedbackQueue] Removed (Gcode_None): {removed}")
            #DEBUG LINE ABOVE
            return
    # Try to remove the specific gcode_type from the queue
        try:
            self.__feedback_queue.remove(gcode_type)
            self.__gcode_state = DeltaX.DONE
            #print(f"[FeedbackQueue] Removed: {gcode_type}, State set to DONE")
            #DEBUG LINE ABOVE
        except ValueError:
            print(f"[FeedbackQueue] Gcode {gcode_type} not found in queue")
  
    def __response_handling(self, response):
        response = response.strip()
        self.__latest_response = response
        print(f"[Response] Received: {response}")

        if response == 'Ok':
            self.__remote_feedback_queue(DeltaX.Gcode_G_M)

        elif response == 'Init Success!':
            pass

        elif response == 'YesDelta':
            self.__is_connected = True 
            self.__is_connecting = False
            self.__remote_feedback_queue(DeltaX.Gcode_Macro)

        elif ':' in response:
            try:
                key, value = response.split(':', 1)

                if key == "Unknown":
                    self.__gcode_state = DeltaX.ERROR
                    self.__remote_feedback_queue(DeltaX.Gcode_None)

                elif key == "Angle":
                    angles = value.split(',')
                    if len(angles) >= 3:
                        self.__real_angle[:len(angles)] = map(float, angles)
                        self.__remote_feedback_queue(DeltaX.Gcode_Macro)

                elif key == "Position":
                    positions = value.split(',')
                    if len(positions) >= 3:
                        self.__real_position[:len(positions)] = map(float, positions)
                        self.__remote_feedback_queue(DeltaX.Gcode_Macro)
            except ValueError:
                print(f"[Warning] Malformed key:value response: {response}")
            
        elif response[0] in "FWUV":
            parts = response.split(' ')
            if len(parts) >= 5:
                self.__remote_feedback_queue(DeltaX.Gcode_G_M)
                for i, part in enumerate(parts):
                    try:
                        val = float(part.split(':')[1])
                        if response[0] == "F":
                            self.__parameter[i] = val
                        elif response[0] == "W":
                            self.__w_parameter[i] = val
                        elif response[0] == "U":
                            self.__u_parameter[i] = val
                        elif response[0] == "V":
                            self.__v_parameter[i] = val
                    except (IndexError, ValueError):
                        print(f"[Warning] Malformed parameter: {part}")


        elif response[0] == "I":
            try:
                index = int(response[1])
                value = int(response[4:])
                self.__i_input[index] = value
                self.__remote_feedback_queue(DeltaX.Gcode_G_M)
            except:
                print(f"[Warning] Malformed I input: {response}")


        elif response[0] == "A":
            try:
                index = int(response[1])
                value = int(response[4:])
                self.__a_input[index] = value
                self.__remote_feedback_queue(DeltaX.Gcode_G_M)
            except:
                print(f"[Warning] Malformed A input: {response}")


        else:
        # Fallback: try to parse as position
            parts = response.split(',')
            if len(parts) >= 3:
                try:
                    self.__real_position[:len(parts)] = map(float, parts)
                    self.__remote_feedback_queue(DeltaX.Gcode_Macro)
                except:
                    print(f"[Warning] Malformed position data: {response}")
            else:
                print(f"[Unhandled] No matching response type: {response}")
          
    def __send_gcode_to_robot(self, data):
        if self.__serial.isOpen() == False:
            return
        data = data + '\n'
        if data[0] == 'G':
            self.__feedback_queue.append(DeltaX.Gcode_G_M)
        elif data[0] == 'M':
            if data[1] == '7':
                data__ = data.split(' ')
                for index in range(0, len(data__) - 1):
                    self.__feedback_queue.append(DeltaX.Gcode_G_M)
            else:
                self.__feedback_queue.append(DeltaX.Gcode_G_M)
        else:
            self.__feedback_queue.append(DeltaX.Gcode_Macro)
        self.__serial.write(data.encode())
    
    def sendGcode(self, data):
        """Send gcode to robot."""
        self.__send_gcode_to_robot(data)

    def polling_wait(self, timeout=20.0, poll_interval=0.1):
        """Wait for the robot to respond."""
        start_time = time.time()
        while len(self.__feedback_queue) != 0:
            #print(self.__feedback_queue)
            if time.time() - start_time > timeout:
                self.handle_timeout()
                #raise TimeoutError("\nRobot did not complete movement within timeout interval: 10 seconds. See robot.py library line 222 for details.\n")
            time.sleep(poll_interval) #eases burden on CPU
        return self.__gcode_state
    
    def robot_response(self):
        """Last response from robot."""
        return self.__latest_response

    def isResponded(self):
        """Return True if robot responded"""
        if len(self.__feedback_queue) > 0:
            return False
        else:
            return True

    def lastGcodeState(self):
        """Return last gcode state"""
        return self.__gcode_state

    def syncMotionParameters(self, axis = AXIS_XYZ):
        """Using for DeltaX S. Get motion parameters from robot."""
        if self.model == DeltaX.DeltaX_V2:
            print("syncMotionParameters: Using for DeltaX S")
            return

        gcode_str = "M220 I"
        gcode_str += str(axis)
        self.__send_gcode_to_robot(gcode_str)
        return

    def motionParameters(self, axis = AXIS_XYZ):
        """Using for DeltaX S. Return motion parameters available in memory."""
        if self.model == DeltaX.DeltaX_V2:
            print("motionParameters: Using for DeltaX S")
            return

        if axis == DeltaX.AXIS_XYZ:
            return self.__parameter
        elif axis == DeltaX.AXIS_W:
            return self.__w_parameter
        elif axis == DeltaX.AXIS_U:
            return self.__u_parameter
        elif axis == DeltaX.AXIS_V:
            return self.__v_parameter

    def sleep(self, time):
        """Pause the robot for a period of time."""
        if time > 0:
            gcode_str = "G04 P"
            gcode_str += str(time)
            self.__send_gcode_to_robot(gcode_str)

    def position(self):
        """Return position available in memory."""
        return self.__real_position

    def angle(self):
        """Return arm angle available in memory."""
        return self.__real_angle

    def homing(self):
        """Auto-home one or more axes, moving them towards their endstops until triggered.."""
        gcode_str = 'G28'
        self.__send_gcode_to_robot(gcode_str)

    def syncPosition(self):
        """Get position from robot."""
        gcode_str = "Position"
        self.__send_gcode_to_robot(gcode_str)

    def syncAngle(self):
        """Get arm angle from robot."""  
        gcode_str = "Angle"
        self.__send_gcode_to_robot(gcode_str)

    def syncInput(self, I = [], A = []):
        """Using for DeltaX S. Read digital and analog input signals from robot."""
        if self.model == DeltaX.DeltaX_V2:
            print("syncInput: Using for DeltaX S")
            return

        gcode_str = "M7"
        if len(I) == 0 and len(A) == 0:
            return

        for index in range(0, len(I)):
            gcode_str += " I" + str(I[index])
        for index in range(0, len(A)):
            gcode_str += " A" + str(A[index])

        self.__send_gcode_to_robot(gcode_str)

    def getDigitalInput(self, I = []):
        """Using for DeltaX S. Return digital input signals available in memory."""
        if self.model == DeltaX.DeltaX_V2:
            print("getDigitalInput: Using for DeltaX S")
            return

        if len(I) == 0:
            return []
        _i = []
        for index in range(0, len(I)):
            _i.append(self.__i_input[I[index]])
        return _i

    def getAnalogInput(self, A = []):
        """Using for DeltaX S. Return analog input signals available in memory."""
        if self.model == DeltaX.DeltaX_V2:
            print("getAnalogInput: Using for DeltaX S")
            return

        if len(A) == 0:
            return []
        _a = []
        for index in range(0, len(A)):
            _a.append(self.__a_input[A[index]])
        return _a

    def setDO(self, D = [], P = [], value = OFF, mode = 8):
        """Using for DeltaX S. This is the command used to turn on or off the Delta X S robot's output pin."""
        if self.model == DeltaX.DeltaX_V2:
            print("setDO: Using for DeltaX S")
            return

        if len(D) == 0 and len(P) == 0:
            return
        gcode_str = ""
        
        if value == DeltaX.OFF:
            gcode_str += "M05"
        elif mode == 8:
            gcode_str += "M03"
            if len(P) != 0:
                gcode_str += " W" + str(value)
            elif value == DeltaX.OFF:
                gcode_str += " W0"
            elif value == DeltaX.ON:
                gcode_str += " W1"

        elif mode == 16:
            gcode_str += "M04"
            if len(P) != 0:
                gcode_str += " W" + str(value)
            elif value == DeltaX.OFF:
                gcode_str += " W0"
            elif value == DeltaX.ON:
                gcode_str += " W1"

        for index in range(0, len(D)):
            gcode_str += " D" + str(D[index])
        for index in range(0, len(P)):
            gcode_str += " P" + str(P[index])

        self.__send_gcode_to_robot(gcode_str)

    def controlEndEffector(self, dir = CW, value = OFF):
        """Using for DeltaX V2. controlEndEffector is used to turn on or off the vacuum pump, laser, and close the gripper."""
        if self.model != DeltaX.DeltaX_V2:
            print("controlEndEffector: Using for DeltaX V2")
            return

        if value == DeltaX.OFF:
            gcode_str = 'M05'
            self.__send_gcode_to_robot(gcode_str)
            return
        elif dir == DeltaX.CW:
            gcode_str = 'M03'
        elif dir == DeltaX.CCW:
            gcode_str = 'M04'

        if value != DeltaX.ON:
            gcode_str += ' S' + str(value)

        self.__send_gcode_to_robot(gcode_str)

    def setEndEffector(self, name = Vacuum):
        """Using for DeltaX V2. Select the end effector for the delta robot."""

        if self.model != DeltaX.DeltaX_V2:
            print("setEndEffector: Using for DeltaX V2")
            return

        gcode_str = 'M360 E'
        gcode_str += str(name)

        self.__send_gcode_to_robot(gcode_str)
    
    def disableSteppers(self):
        """This command can be used to disable steppers."""
        gcode_str = "M84"
        self.__send_gcode_to_robot(gcode_str)

    def setAcceleration(self, accel):
        """Set the acceleration for moving base of robot."""
        if accel > 0:
            self.__parameter[DeltaX.ROBOT_A] = accel
            gcode_str = "M204 A"
            gcode_str += str(accel)
            self.__send_gcode_to_robot(gcode_str)

    def setStartingAndEndingSpeeds(self, speed):
        """Set the starting and ending speeds for each movement of the robot."""
        if speed > 0:
            self.__parameter[DeltaX.ROBOT_VS] = speed
            self.__parameter[DeltaX.ROBOT_VE] = speed
            gcode_str = "M205 S"
            gcode_str += str(speed)
            self.__send_gcode_to_robot(gcode_str)

    def setXYZOffset(self, point = []):
        """Use setXYZOffset to apply a persistent X Y Z offset to the native home position and coordinate space.
        This effectively shifts the coordinate space in the negative direction."""

        gcode_str = 'M206'
        gcode_str += " X" + str(point[0])
        gcode_str += " Y" + str(point[1])
        gcode_str += " Z" + str(point[2])
        self.__send_gcode_to_robot(gcode_str)

    def moveL(self, point = [], velocity = 0.0, accel = 0.0, begin_vel = -1.0, end_vel = -1.0):
        """The moveL commands add a linear MOVE to the queue to be performed after all previous moves are completed.
        A command like G1 F1000 sets the feed rate for all subsequent moves."""

        gcode_str = 'G1'
        gcode_str += ' X' + str(point[0])
        gcode_str += ' Y' + str(point[1])
        gcode_str += ' Z' + str(point[2])
        if len(point) > 3:
            gcode_str += ' W' + str(point[3])
        if velocity != 0.0:
            self.__parameter[DeltaX.ROBOT_V] = velocity
            gcode_str += ' F' + str(velocity)
        if self.model == DeltaX.DeltaX_S:    
            if accel != 0.0:
                self.__parameter[DeltaX.ROBOT_A] = accel
                gcode_str += ' A' + str(accel)
            if begin_vel != self.__parameter[DeltaX.ROBOT_VS] and begin_vel > 0:
                self.__parameter[DeltaX.ROBOT_VS] = begin_vel
                gcode_str += ' S' + str(begin_vel)
            if end_vel != self.__parameter[DeltaX.ROBOT_VE] and end_vel > 0:
                self.__parameter[DeltaX.ROBOT_VE] = end_vel
                gcode_str += ' E' + str(end_vel)

        self.__send_gcode_to_robot(gcode_str)
    
    def moveC(self, dir = CW, offset = [], point = [], velocity = 0.0, accel = 0.0, begin_vel = -1.0, end_vel = -1.0):
        """CW adds a clockwise arc move to the planner; CWW adds a counter-clockwise arc.
        An arc move starts at the current position and ends at the given XYZ, pivoting around a center-point offset given by I and J."""

        gcode_str = ""
        if dir == DeltaX.CW:
            gcode_str += "G2"
        elif dir == DeltaX.CCW:
            gcode_str += "G3"

        gcode_str += ' I' + str(offset[0])
        gcode_str += ' J' + str(offset[1])
        gcode_str += ' X' + str(point[0])
        gcode_str += ' Y' + str(point[1])
        if len(point) > 2:
            gcode_str += ' W' + str(point[2])
        if velocity != 0.0:
            self.__parameter[DeltaX.ROBOT_V] = velocity
            gcode_str += ' F' + str(velocity)
        if self.model == DeltaX.DeltaX_S: 
            if accel != 0.0:
                self.__parameter[DeltaX.ROBOT_A] = accel
                gcode_str += ' A' + str(accel)
            if begin_vel != self.__parameter[DeltaX.ROBOT_VS] and begin_vel > 0:
                self.__parameter[DeltaX.ROBOT_VS] = begin_vel
                gcode_str += ' S' + str(begin_vel)
            if end_vel != self.__parameter[DeltaX.ROBOT_VE] and end_vel > 0:
                self.__parameter[DeltaX.ROBOT_VE] = end_vel
                gcode_str += ' E' + str(end_vel)

        self.__send_gcode_to_robot(gcode_str)
        
    def send(self, gCode, timeout = 5.0, poll_interval = 0.05):
        print(f"Sending gCode: {gCode}")
        self.__send_gcode_to_robot(gCode)
        
        start_time = time.time()
        while not self.isResponded():
            if time.time() - start_time > timeout:
                self.handle_timeout()
                #raise TimeoutError("\nRobot did not complete movement within timeout interval: 5 seconds. See robot.py library line 505 for details.\n")
            time.sleep(poll_interval)
        response = self.robot_response()
        print(f"Robot responded: {response}")
        return response
            
    #FLUSH THE QUEUE
    def handle_timeout(self):
        print("[Recovery] Timeout occurred. Resetting state and flushing buffer.")
        self.__gcode_state = DeltaX.ERROR
        self.__remote_feedback_queue(DeltaX.Gcode_None)
        #self.__gcode_state = DeltaX.DONE
        #self.__feedback_queue.clear()
        #self.__serial.reset_input_buffer()

    def print_feedback(self):
        print(self.__feedback_queue)
