import serial
import serial.tools.list_ports
import struct
import time
import crcmod
import threading

# Packet structure definition
PACKET_IDENTIFY = b'\xAA\xCC\xFF\x99'
PACKET_SIZE = 4 + 2 + (5 * 2) + 4  # identify + type + data[5] + crc
PAYLOAD_SIZE = PACKET_SIZE - len(PACKET_IDENTIFY)

# Packet types
TYPE_HOME = 0
TYPE_GOTO = 1
TYPE_MOVE = 2
TYPE_BAG = 3
TYPE_STATUS = 4
TYPE_NAME = 5

class SerialPacketController:
    def __init__(self, target: str, baudrate: int):
        self._ser = None
        ## get serial port
        serial_ok = False
        ports = serial.tools.list_ports.comports()
        available_ports = []
        for port, desc, hwid in sorted(ports):
            ## get ttyACM or ttyUSB
            # print(port)
            c = port[8]
            # print(c)
            if c == 'A' or c == 'U':
                available_ports.append(port)
        # available_ports = ['/dev/ttyUSB0', '/dev/ttyACM0']
        available_ports = ['/dev/ttyUSB0']
        print(f"available ports: {available_ports}")
        ## try port
        for port in available_ports:
            try:
                print(f"Connecting to {port}")
                self._ser = serial.Serial(port, baudrate, timeout = 3)
                received_name = False
                start_time = time.time()
                while(time.time() - start_time < 3):
                    self._ser.read_until(PACKET_IDENTIFY)
                    raw_data = self._ser.read(PAYLOAD_SIZE)
                    # print(f"raw: {raw_data}")
                    p_type, device_name, received_crc = struct.unpack('<h10sL', raw_data)
                    if p_type == TYPE_NAME:
                        device_name = device_name.decode('ascii')
                        print(f"Device name: {device_name}")
                        received_name = True
                        break
                # device_name = self._ser.readline().replace("\r\n", "")
                ## name match
                if received_name and device_name == target:
                    serial_ok = True
                    print("Serial open success")
                    break
                else:
                    print("Not this port")
                    self._ser.close()
            except Exception as e:
                # print(f"Serail Exception: {e}")
                continue
        # exit()
        ## exit when serial not opened
        if not serial_ok:
            raise Exception("Serial port not opened")
        print(f"Serial open success: {self._ser.name}")
        self._crc32_func = crcmod.predefined.mkCrcFun('crc-32')
        self._response_mutex = threading.Lock()
        self._response_packets: dict = None
        self._status_mutex = threading.Lock()
        self._status_packets: list = None
        self._receive_thread = None
        self._status_callback: callable = None

    def _send_packet(self, packet_type: int, data: list = None) -> None:
        '''
        Send packet to serial port\n
        :param packet_type: packet type
        :param data: packet data
        '''
        if data is None:
            data = [0] * 5
        elif len(data) < 5:
            data.extend([0] * (5 - len(data)))
        # Pack without CRC initially
        # Format: 4s (identify) + h (type) + 5h (data)
        packet_without_crc = struct.pack('<4sh5h', PACKET_IDENTIFY, packet_type, *data[:5])
        crc = self._crc32_func(packet_without_crc)
        # Pack with CRC
        # Format: 4s (identify) + h (type) + 5h (data) + L (crc)
        full_packet = struct.pack('<4sh5hL', PACKET_IDENTIFY, packet_type, *data[:5], crc)
        # Send the full packet
        self._ser.write(full_packet)

    def _receive_thread_function(self):
        while True:
            ## read from serial
            try:
                ## read until header
                self._ser.read_until(PACKET_IDENTIFY)
                raw_data = self._ser.read(PAYLOAD_SIZE)
            except Exception as e:
                print("Error reading from serial port:", e)
                continue
            ## unpack
            try:
                p_type, d0, d1, d2, d3, d4, received_crc = struct.unpack('<h5hL', raw_data)
            except struct.error:
                print("Error unpacking received packet in thread.")
                continue
            ## Verify CRC
            packet_without_crc = struct.pack('<4sh5h', PACKET_IDENTIFY, p_type, d0, d1, d2, d3, d4)
            calculated_crc = self._crc32_func(packet_without_crc)
            if received_crc == calculated_crc:
                packet_data = {
                    "type": p_type,
                    "data": [d0, d1, d2, d3, d4],
                }
                if p_type == TYPE_STATUS:
                    self._status_mutex.acquire()
                    self._status_packets = packet_data["data"]
                    self._status_mutex.release()
                    if self._status_callback is not None:
                        self._status_callback()
                elif p_type == TYPE_NAME:
                    pass
                else:
                    self._response_mutex.acquire()
                    self._response_packets = packet_data
                    self._response_mutex.release()
            else:
                print(f"CRC Mismatch in thread: Received {received_crc:#010x}, Calculated {calculated_crc:#010x}")
            ## loop delay
            time.sleep(0.001) # Prevent busy-waiting on errors

    def start_receiving(self) -> None:
        '''
        Start receiving thread
        '''
        if self._receive_thread is None:
            self._receive_thread = threading.Thread(target=self._receive_thread_function)
            self._receive_thread.daemon = True  # Allow the main program to exit even if thread is running
            self._receive_thread.start()

    def _get_response(self) -> dict | None:
        '''
        Get the latest response packet
        `return` None when no packet is available
        '''
        self._response_mutex.acquire()
        packet = self._response_packets
        self._response_mutex.release()
        return packet

    def _get_status(self) -> list | None:
        '''
        Get the raw status packet\n
        `return` None when no packet is available
        '''
        self._status_mutex.acquire()
        packet = self._status_packets
        self._status_mutex.release()
        return packet

    def close(self) -> None:
        '''
        Close serial port
        '''
        if self._ser is not None:
            self._ser.close()

    def _wait_response(self, packet_type: int, timeout=5) -> bool:
        '''
        Wait until response packet is received\n
        :param packet_type: packet type
        :param timeout: seconds
        :return: True when response packet is received; False when timeout
        '''
        start_time = time.time()
        while time.time() - start_time < timeout:
            packet = self._get_response()
            if packet:
                # print(f"resp: {packet}")
                pass
            if packet and packet["type"] == packet_type:
                return True
            time.sleep(0.1)
        return False

    def _wait_not_busy(self, timeout=8) -> bool:
        '''
        Wait until not busy\n
        :param timeout: seconds
        :return: True when not busy; False when timeout
        '''
        time.sleep(0.2)
        start_time = time.time()
        while time.time() - start_time < timeout:
            packet = self._get_status()
            # print(f"busy: {self._status_packets[3]}")
            if packet and not packet[3]:
                return True
            time.sleep(0.1)
        return False

    def goto(self, position: float, velocity: float) -> bool:
        '''
        Control absolute move\n
        :param position: mm
        :param velocity: mm/s
        :return: True when success
        '''
        failed = True
        for _ in range(3):
            self._send_packet(TYPE_GOTO, [position*10, velocity*10])
            if self._wait_response(TYPE_GOTO):
                failed = False
                break
        ## return false when retry failed
        if failed:
            print("goto not response")
            return False
        self._wait_not_busy()
        return True

    def move(self, distance: float, velocity: float) -> bool:
        '''
        Control relative move\n
        :param distance: mm
        :param velocity: mm/s
        :return: True when success
        '''
        failed = True
        for _ in range(3):
            self._send_packet(TYPE_MOVE, [distance*10, velocity*10])
            if self._wait_response(TYPE_MOVE):
                failed = False
                break
        ## return false when retry failed
        if failed:
            print("move not response")
            return False
        self._wait_not_busy()
        return True

    def home(self) -> bool:
        '''
        Control go to home\n
        :return: True when success
        '''
        failed = True
        for _ in range(3):
            self._send_packet(TYPE_HOME)
            if self._wait_response(TYPE_HOME):
                failed = False
                break
        ## return false when retry failed
        if failed:
            print("home not response")
            return False
        self._wait_not_busy()
        return True

    def bag(self, deg: int) -> bool:
        '''
        Control bag servo rotation\n
        :param deg: 0-180
        :return: True when success
        '''
        failed = True
        for _ in range(3):
            self._send_packet(TYPE_BAG, [deg])
            if self._wait_response(TYPE_BAG):
                failed = False
                break
        ## return false when retry failed
        if failed:
            print("bag not response")
            return False
        return True

    def status(self) -> tuple[float, float, bool, bool, bool] | None:
        '''
        Read latest status\n
        :return: (position, velocity, homed, busy, bag_detected); None when no packet is available
        '''
        packet = self._get_status()
        if packet is not None:
            return packet[0]/10, packet[1]/10, bool(packet[2]), bool(packet[3]), bool(packet[4])
        return None

    def set_status_callback(self, callback: callable) -> None:
        '''
        Set status callback function\n
        :param callback: will been called when receive status pack, 
            must call status() manually to get status
        '''
        self._status_callback = callback

    def auto_bag(self, release_deg: int, lock_deg: int) -> None:
        '''
        Release bag and wait bag detected, then lock bag\n
        '''
        controller.bag(release_deg)
        while True:
            status = controller.status()
            if status is not None:
                position, velocity, homed, busy, bag_detect = status
                if bag_detect:
                    time.sleep(0.2)
                    controller.bag(lock_deg)
                    time.sleep(1)
                    break



# Example Usage
if __name__ == "__main__":
    baud_rate = 115200
    controller = None
    try:
        controller = SerialPacketController("arm_rail1", baud_rate)
        controller.start_receiving() # Start the receiving thread
        time.sleep(1)
        print("init")
        # while 1:
        #     time.sleep(0.1)
        #     status = controller.status()
        #     if status is not None:
        #         print(status)
        #     pass
        BAG_RELEASE_DEG = 170
        BAG_LOCK_DEG = 0
        controller.home()
        controller.bag(BAG_RELEASE_DEG)
        # controller.goto(1800, 300)
        print("start loop")
        while True:
            status = controller.status()
            if status is not None:
                position, velocity, homed, busy, bag_detect = status
                ## test code
                if bag_detect:
                    controller.bag(BAG_LOCK_DEG)
                    time.sleep(0.5)
                    controller.goto(1900, 300)
                    print("end go")
                    time.sleep(1)
                    controller.goto(10, 300)
                    print("end back")
                    time.sleep(1)
                    controller.bag(BAG_RELEASE_DEG)
                    time.sleep(1)
            time.sleep(0.1) # Adjust this sleep to control how often the main thread checks for new packets
    ## Exception
    except serial.SerialException as e:
        print(f"Serial Port Error: {e}")
    except KeyboardInterrupt:
        print("\nExiting.")
    finally:
        if controller:
            controller.close()
            print("Serial port closed.")