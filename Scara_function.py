# -*- coding:utf-8 -*-

import serial
import binascii
import time
import math

class Scara():
    # CRC16 Table
    crcTable = [0x0000,0xC0C1,0xC181,0x0140,0xC301,0x03C0,0x0280,0xC241,0xC601,0x06C0,
              0x0780,0xC741,0x0500,0xC5C1,0xC481,0x0440,0xCC01,0x0CC0,0x0D80,0xCD41,
              0x0F00,0xCFC1,0xCE81,0x0E40,0x0A00,0xCAC1,0xCB81,0x0B40,0xC901,0x09C0,
              0x0880,0xC841,0xD801,0x18C0,0x1980,0xD941,0x1B00,0xDBC1,0xDA81,0x1A40,
              0x1E00,0xDEC1,0xDF81,0x1F40,0xDD01,0x1DC0,0x1C80,0xDC41,0x1400,0xD4C1,
              0xD581,0x1540,0xD701,0x17C0,0x1680,0xD641,0xD201,0x12C0,0x1380,0xD341,
              0x1100,0xD1C1,0xD081,0x1040,0xF001,0x30C0,0x3180,0xF141,0x3300,0xF3C1,
              0xF281,0x3240,0x3600,0xF6C1,0xF781,0x3740,0xF501,0x35C0,0x3480,0xF441,
              0x3C00,0xFCC1,0xFD81,0x3D40,0xFF01,0x3FC0,0x3E80,0xFE41,0xFA01,0x3AC0,
              0x3B80,0xFB41,0x3900,0xF9C1,0xF881,0x3840,0x2800,0xE8C1,0xE981,0x2940,
              0xEB01,0x2BC0,0x2A80,0xEA41,0xEE01,0x2EC0,0x2F80,0xEF41,0x2D00,0xEDC1,
              0xEC81,0x2C40,0xE401,0x24C0,0x2580,0xE541,0x2700,0xE7C1,0xE681,0x2640,
              0x2200,0xE2C1,0xE381,0x2340,0xE101,0x21C0,0x2080,0xE041,0xA001,0x60C0,
              0x6180,0xA141,0x6300,0xA3C1,0xA281,0x6240,0x6600,0xA6C1,0xA781,0x6740,
              0xA501,0x65C0,0x6480,0xA441,0x6C00,0xACC1,0xAD81,0x6D40,0xAF01,0x6FC0,
              0x6E80,0xAE41,0xAA01,0x6AC0,0x6B80,0xAB41,0x6900,0xA9C1,0xA881,0x6840,
              0x7800,0xB8C1,0xB981,0x7940,0xBB01,0x7BC0,0x7A80,0xBA41,0xBE01,0x7EC0,
              0x7F80,0xBF41,0x7D00,0xBDC1,0xBC81,0x7C40,0xB401,0x74C0,0x7580,0xB541,
              0x7700,0xB7C1,0xB681,0x7640,0x7200,0xB2C1,0xB381,0x7340,0xB101,0x71C0,
              0x7080,0xB041,0x5000,0x90C1,0x9181,0x5140,0x9301,0x53C0,0x5280,0x9241,
              0x9601,0x56C0,0x5780,0x9741,0x5500,0x95C1,0x9481,0x5440,0x9C01,0x5CC0,
              0x5D80,0x9D41,0x5F00,0x9FC1,0x9E81,0x5E40,0x5A00,0x9AC1,0x9B81,0x5B40,
              0x9901,0x59C0,0x5880,0x9841,0x8801,0x48C0,0x4980,0x8941,0x4B00,0x8BC1,
              0x8A81,0x4A40,0x4E00,0x8EC1,0x8F81,0x4F40,0x8D01,0x4DC0,0x4C80,0x8C41,
              0x4400,0x84C1,0x8581,0x4540,0x8701,0x47C0,0x4680,0x8641,0x8201,0x42C0,
              0x4380,0x8341,0x4100,0x81C1,0x8081,0x4040]
    warningTable = {'01':'Main power fail', '02':'Low encoder battery',
                  '04':'SW position limit', '08':'Over DB current',
                  '10':'Over load', '20':'Setup check',
                  '40':'Under voltage', '80':'EMG', '00':'No warning'}
    # Define protocol(without id, crc)
    SV_ON = "\x05\x00\x0C\xFF\x00"
    SV_OFF = "\x05\x00\x0C\x00\x00"
    START_ON = "\x05\x00\x10\xFF\x00"
    START_OFF = "\x05\x00\x10\x00\x00"
    PAUSE_ON = "\x05\x00\x11\xFF\x00"
    PAUSE_OFF = "\x05\x00\x11\x00\x00"
    STOP_ON = "\x05\x00\x03\xFF\x00"
    STOP_OFF = "\x05\x00\x03\x00\x00"
    EMG_ON = "\x05\x00\x0A\xFF\x00"
    EMG_OFF = "\x05\x00\x0A\x00\x00"
    A_RESET_ON = "\05\x00\x0B\xFF\x00"
    A_RESET_OFF = "\05\x00\x0B\x00\x00"
    STATUS = "\x04\x21\x21\x00\x01"

    JOG = "\x10\x23\x00\x00\x03\x06\xF6\xA0\x00\x64\x00\x64"
    R_POSITION = "\x03\x26\x36\x00\x02"
    R_WARN = "\x03\x26\x1B\x00\x01"
    R_ALRAM = "\x04\x27\x07\x00\x01"
    INDEX_ACTION = "\x10\x31\x11\x00\x01\x02\x00\x00"
    STOP_DECEL = "\x10\x30\x36\x00\x02\x04"

    def __init__(self):
        # Set serial port
        self.port = serial.Serial(port='/dev/SCARA1', baudrate=57600, timeout=0, parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,bytesize=serial.EIGHTBITS)

    def unsigned32(self, n):
        return n & 0xFFFFFFFF

    def index_setting(self, node_id):
        # Index action param setting(Action: 0))
        for i in node_id:
            packet = self.packet_generator(i, self.INDEX_ACTION)
            self.port.write(packet)
            time.sleep(0.02)
            data_raw = self.port.readline(self.port.in_waiting)

    def inverse_calc(self, x, y):
        l1 = 350
        l2 = l1
        ctheta2 = (x**2 + y**2 - l1**2 - l2**2)/(2 * l1 * l2)
        stheta2_n = -(math.sqrt(1-ctheta2**2))
        stheta2_p = math.sqrt(1-ctheta2**2)

        th2_n = math.atan2(stheta2_n, ctheta2)
        th2_p = math.atan2(stheta2_p, ctheta2)
        th1_n = math.atan2(y, x) - math.atan2((l2*stheta2_n), (l1+l2*ctheta2))
        th1_p = math.atan2(y, x) - math.atan2((l2*stheta2_p), (l1+l2*ctheta2))

        th2_n = math.degrees(th2_n)
        th2_p = math.degrees(th2_p)
        th1_n = math.degrees(th1_n)
        th1_p = math.degrees(th1_p)
        # n: 1,4 quadrant  p: 2,3 quadrant

        if y > 0:
            theta1 = th1_p
            theta2 = th2_p
        elif y <= 0:
            theta1 = th1_n
            theta2 = th2_n

        return theta1, theta2

    def check_workspace(self, x, y):
        check1 = True
        if (math.sqrt(x ** 2 + y ** 2) > 700) or (math.sqrt(x ** 2 + y ** 2) < 100):
            check1 = False

        return check1
        

    def position_cmd(self, raw_pulse, init_pulse, current_deg, target_degree, operating_time, node_id):
        # Limit : +-126 degree
        # Calcuate velocity
        if node_id == 1:
            one_degree = (262144*80)/360
        else:
            one_degree = (262144*40)/360
        # one_degree = 262144
        current_pulse = raw_pulse
        target_pulse = int(round(target_degree * one_degree) + init_pulse)
        
        relative_dist = abs(target_pulse - current_pulse)
        velocity = int(math.ceil(relative_dist/(0.58*operating_time)))
        accel = int(math.ceil(velocity/(0.4*operating_time)))
        decel = int(math.ceil(velocity/(0.4*operating_time)))
        print("velocity:", velocity)
        accel_hex = hex(accel)
        decel_hex = hex(decel)
        accel_str = str(accel_hex)[2:].zfill(8)
        decel_str = str(accel_hex)[2:].zfill(8)

        # Additional cmd str
        header_str = '103101000912'
        index_str = '0000'
        # int to hex
        if target_pulse >= 0:
            dist_str = hex(target_pulse)[2:].zfill(8)
        else:
            unsigned_data = self.unsigned32(target_pulse)
            dist_str = hex(unsigned_data)[2:].zfill(8)

        if velocity >= 0:
            vel_str = hex(velocity)[2:].zfill(8)
        else:
            print("Velocity value error")

        # swap register data component
        dist_str_swap = ''
        vel_str_swap = ''
        accel_str_swap = ''
        decel_str_swap = ''
        for i in range(2):
            i *= 4
            dist_str_swap = dist_str[i]+dist_str[i+1]+dist_str[i+2]+dist_str[i+3] + dist_str_swap
            vel_str_swap = vel_str[i]+vel_str[i+1]+vel_str[i+2]+vel_str[i+3] + vel_str_swap
            accel_str_swap = accel_str[i]+accel_str[i+1]+accel_str[i+2]+accel_str[i+3] + accel_str_swap
            decel_str_swap = decel_str[i]+decel_str[i+1]+decel_str[i+2]+decel_str[i+3] + decel_str_swap

        position_cmd = header_str + index_str + dist_str_swap + vel_str_swap + accel_str_swap + decel_str_swap
        position_cmd = binascii.unhexlify(position_cmd)

        stop_decel_str = binascii.hexlify(self.STOP_DECEL)
        stop_decel_cmd = stop_decel_str + decel_str_swap
        stop_decel_cmd = binascii.unhexlify(stop_decel_cmd)
        return position_cmd, stop_decel_cmd
 
    def read_position_data(self, data):
        id = data[0] + data[1]
        func_code = data[2] + data[3]
        data_len = data[4] + data[5]
        crc_M = data[-4] + data[-3]
        crc_L = data[-2] + data[-1]
        recv_data = data[6:-4]
        num_of_register = len(recv_data) / 4
        recv_data_swap_str = ''

        for i in range(num_of_register):
            i *= 4
            recv_data_swap_str = recv_data[i]+recv_data[i+1]+recv_data[i+2]+recv_data[i+3] + recv_data_swap_str
        recv_data_swap_temp = int(recv_data_swap_str, 16)
        sign_bit = int(bin(recv_data_swap_temp)[2:].zfill(32)[0])

        if sign_bit == 0:
            recv_data_swap = recv_data_swap_temp
        else:
            temp_val = bin(recv_data_swap_temp ^ 0xffffffff)
            recv_data_swap = ~int(temp_val, 2)
        
        recv_data_hex = hex(recv_data_swap)

        return recv_data_swap, recv_data_hex

    def packet_generator(self, id, data):
        id_str = binascii.hexlify(bytearray([id]))
        data_str = binascii.hexlify(data)

        crc_input = binascii.unhexlify(id_str+data_str)
        crc = self.crc16(crc_input)
        crc_str =  binascii.hexlify(bytearray(crc))
        term_str = binascii.hexlify(bytearray([0, 0, 0, 0]))
        packet_str = id_str + data_str + crc_str
        packet = binascii.unhexlify(packet_str)
        return packet

    def crc16(self, data):
        crc= [0xff, 0xff]
        for datum in bytearray(data):
            ncrc = self.crcTable[(crc[0] ^ datum)]
            crc[0] = (ncrc & 0x00FF) ^ crc[1]
            crc[1] = ncrc >> 8
        return crc

    def crc16_recv(self, data):
        recv_crc = data[-4:]
        recv_data = data[:-4]
        crc= [0xff, 0xff]
        crc_input = binascii.unhexlify(recv_data)
        for datum in bytearray(crc_input):
            ncrc = self.crcTable[(crc[0] ^ datum)]
            crc[0] = (ncrc & 0x00FF) ^ crc[1]
            crc[1] = ncrc >> 8
        crc_str =  binascii.hexlify(bytearray(crc))

        if crc_str == recv_crc:
            return True
        else:
            return False
        
    def check_ready(self, node_id):
        if node_id == 0:
            result = []
            for i in range(3):
                status_dict = self.read_status(i+1)
                if (status_dict['servo']==0 and status_dict['alarm']==0 and status_dict['warn']==0
                    and status_dict['ready']==1 and status_dict['inpos1']==1 and status_dict['zspd']==1):
                    result.append(True)
                else:
                    result.append(False)
            return result[0] and result[1] and result[2]

        else:
            status_dict = self.read_status(node_id)

            if (status_dict['servo']==0 and status_dict['alarm']==0 and status_dict['warn']==0
                and status_dict['ready']==1 and status_dict['inpos1']==1 and status_dict['zspd']==1):
                return True

            else:
                return False

    def check_recv(self, i):
        t1 = time.time()
        while True:
            temp = self.port.in_waiting
            if temp >= i:
                data_raw = self.port.read_all()
                return data_raw
            t2 = time.time()
            if t2-t1 > 0.5:
                print("recv_fail")
                print(temp)
                data_raw = self.port.read_all()
                return data_raw

    ####################################
    ######## Operating Function ########
    ####################################
    def alarm_reset(self, node_id):
        packet = self.packet_generator(node_id, self.A_RESET_ON)
        self.port.write(packet)
        data_raw = self.check_recv(8)

        packet = self.packet_generator(node_id, self.A_RESET_OFF)
        self.port.write(packet)
        data_raw = self.check_recv(8)

    def scara_alarm(self, node_id):
        packet = self.packet_generator(node_id, self.R_ALRAM)
        self.port.write(packet)
        data_raw = self.check_recv(8)
        data = binascii.hexlify(data_raw)
        return data

    def emg_on(self, node_id):
        packet = self.packet_generator(node_id, self.EMG_ON)
        self.port.write(packet)
        data_raw = self.check_recv(8)

    def emg_off(self, node_id):
        packet = self.packet_generator(node_id, self.EMG_OFF)
        self.port.write(packet)
        data_raw = self.check_recv(8)

    def sv_on(self, node_id):
        # node_id: int
        t1 = time.time()
        self.port.flushInput()
        packet = self.packet_generator(node_id, self.SV_ON)
        self.port.write(packet)
        data_raw = self.check_recv(8)
        t2 = time.time()
        print("recv :",t2-t1, binascii.hexlify(data_raw))

    def sv_off(self, node_id):
        t1 = time.time()
        self.port.flushInput()
        packet = self.packet_generator(node_id, self.SV_OFF)
        self.port.write(packet)
        data_raw = self.check_recv(8)
        t2 = time.time()
        print("recv :",t2-t1, binascii.hexlify(data_raw))

    def start(self, node_id):
        # start on
        t1 = time.time()
        self.port.flushInput()
        packet = self.packet_generator(node_id, self.START_ON)
        self.port.write(packet)
        time.sleep(0.002)
        data_raw = self.port.readline(self.port.in_waiting)
        # start off
        packet = self.packet_generator(node_id, self.START_OFF)
        self.port.write(packet)
        time.sleep(0.002)
        #data_raw = self.check_recv(8)
        t2 = time.time()
        #print("start", t2-t1)
        data_raw = self.port.readline(self.port.in_waiting)
        # print(binascii.hexlify(data_raw))

    def stop(self, node_id):
        # stop on
        packet = self.packet_generator(node_id, self.STOP_ON)
        self.port.write(packet)
        data_raw = self.check_recv(8)
        # stop off
        packet = self.packet_generator(node_id, self.STOP_OFF)
        self.port.write(packet)
        data_raw = self.check_recv(8)

    def pause(self, node_id):
        # pause on
        packet = self.packet_generator(node_id, self.PAUSE_ON)
        self.port.write(packet)
        data_raw = self.check_recv(8)
        # pause off
        packet = self.packet_generator(node_id, self.PAUSE_OFF)
        self.port.write(packet)
        data_raw = self.check_recv(8)

    def read_status(self, node_id):
        # bit 8:warn, 4:inpos1, 3:zspd, 2:ready, 1:alarm, 0:break
        while True:
            self.port.flushInput()
            packet = self.packet_generator(node_id, self.STATUS)
            self.port.write(packet)
            data_raw = self.check_recv(7)
            data = binascii.hexlify(data_raw)
            if self.crc16_recv(data):
                data_bin = bin(int(data[6:-4], 16))
                self.port.flushInput()
                break
            else:
                pass
        data_bin = data_bin[2:].zfill(11)
        data_bin_list = list(data_bin)
        data_bin_list.reverse()
        for i in range(len(data_bin_list)):
            data_bin_list[i] = int(data_bin_list[i])

        # servo : 0(ON), 1(OFF), others : active high
        status_dict = {'servo':data_bin_list[0], 'alarm':data_bin_list[1], 'ready':data_bin_list[2],
                       'zspd':data_bin_list[3],'inpos1':data_bin_list[4], 'warn':data_bin_list[8]}
        return status_dict

    def read_position(self, node_id):
        self.port.flushInput()
        packet = self.packet_generator(node_id, self.R_POSITION)
        self.port.write(packet)
        data_raw = self.check_recv(9)
        data = binascii.hexlify(data_raw)
        if self.crc16_recv(data):
            dec_val, hex_val = self.read_position_data(data)
            return dec_val, hex_val
        else:
            return None
    
    def joint_control(self, raw_pulse, init_pulse, current_deg, node_id, degree, operating_t):
        t1 = time.time()
        if node_id == 0:
            print("\n[WARN] Invalid Node ID\n")
        else:
            # 1 turn = 262144
            # degree: float, operating_t: float
            try:
                position_cmd, stop_decel_cmd = self.position_cmd(raw_pulse, init_pulse, current_deg, degree, operating_t, node_id)
            except:
                print("position control None")
                return None
            t2 = time.time()
            self.port.flushInput()
            packet = self.packet_generator(node_id, position_cmd)
            self.port.write(packet)
            data_raw = self.check_recv(8)
            data = binascii.hexlify(data_raw)
            
            t3 = time.time()
            self.port.flushInput()
            packet = self.packet_generator(node_id, stop_decel_cmd)
            self.port.write(packet)
            data_raw = self.check_recv(8)
            t4 = time.time()

    def flush(self):
        self.port.flushInput()
