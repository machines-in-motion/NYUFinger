import socket
import struct
import numpy as np
import time
import threading
from FingerMsgs import FingerCmd, FingerState
from cyclonedds.domain import DomainParticipant
from cyclonedds.topic import Topic
from cyclonedds.pub import DataWriter
from cyclonedds.sub import DataReader
from dataclasses import dataclass

@dataclass
class RobotConfig:
    robot_ip: str
    robot_port: int
    local_port: int
    dds_freq: float
    DoF: int
    robot_name: str
    state_fields: list
    command_fields: list
    current2Torque: float
    gear_ratio: float
    max_torque: float

class TeensyUDPBridge():
    def __init__(self, 
                 cfg, 
                 user_callback=None):
        self.bridge_ip = cfg.robot_ip
        self.bridge_port = cfg.robot_port
        self.socket=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        self.RX_RUNNING = True
        self.socket.bind(('0.0.0.0', cfg.local_port))
        self.rx_thread=threading.Thread(target=self.receivingThread)
        self.state = None
        self.latest_state_stamp = time.time()
        self.packet = None
        self.user_callback = user_callback
        self.rx_thread.start()

    def sendCommand(self, cmd):
        msg_format=f'{len(cmd)}f'
        payload = cmd
        arguments = [msg_format] + payload
        packet=struct.pack(*arguments)
        self.socket.sendto(packet, (self.bridge_ip, self.bridge_port))

    def getLatestState(self):
        return self.latest_state_stamp, self.state

    def receivingThread(self):
        while self.RX_RUNNING:
            packet = self.socket.recvmsg(4096)[0]
            state_msg_format=f'{len(packet)//4}f'
            data = struct.unpack(state_msg_format, packet)
            self.state = data
            self.latest_state_stamp = time.time()
            # Notify user if a callback is provided
            if self.user_callback is not None:
                print('Calling user callback')
                self.user_callback()

    def close(self):
        self.RX_RUNNING = False
        self.socket.close()
        self.rx_thread.join()


def get_last_msg(reader, topic_type):
    """ """
    last_msg = reader.take()

    if last_msg:
        while True:
            a = reader.take()
            if not a:
                break
            else:
                last_msg = a
    if last_msg:
        msg = last_msg[0]
        if type(msg) == topic_type:
            return msg
        else:
            return None

    else:
        return None
    

class UDPCycloneDDSBridge:
    def __init__(self, udp_bridge: TeensyUDPBridge, cfg: RobotConfig):

        self.cfg = cfg
        self.state_topic_name = f'{cfg.robot_name}_robot_state'
        self.cmd_topic_name = f'{cfg.robot_name}_robot_cmd'
        self.q_idx = []
        self.dq_idx = []
        self.tau_idx = []
        self.kp_idx = []
        self.kd_idx = []
        self.q_des_idx = []
        self.dq_des_idx = []
        self.tau_ff_idx = []
        
        for i, state_name in enumerate(cfg.state_fields):
            if state_name.split('_')[0] == 'q':
                self.q_idx.append(i)
            elif state_name.split('_')[0] == 'dq':
                self.dq_idx.append(i)
            elif state_name.split('_')[0] == 'tau':
                self.tau_idx.append(i)

        for i, command_name in enumerate(cfg.command_fields):
            if command_name.split('_')[0] == 'kp':
                self.kp_idx.append(i)
            elif command_name.split('_')[0] == 'kd':
                self.kd_idx.append(i)
            elif command_name.split('_')[0] == 'q':
                self.q_des_idx.append(i)
            elif command_name.split('_')[0] == 'dq':
                self.dq_des_idx.append(i)
            elif command_name.split('_')[0] == 'tau':
                self.tau_ff_idx.append(i)

        self.udp_bridge = udp_bridge
        self.participant = DomainParticipant()
        self.cmd_topic = Topic(self.participant,self.cmd_topic_name, FingerCmd)
        self.state_topic = Topic(self.participant,self.state_topic_name, FingerState)
        self.cmd_reader = DataReader(self.participant, self.cmd_topic)
        self.state_writer = DataWriter(self.participant, self.state_topic)
        self.udp_cmd = np.zeros(len(cfg.command_fields))

    def forwardUDPStates2DDS(self):
        stamp, teensy_state = self.udp_bridge.getLatestState()
        if teensy_state is not None:
            state = np.array(teensy_state)
            q = (state[self.q_idx]/self.cfg.gear_ratio).tolist()
            dq = (state[self.dq_idx]/self.cfg.gear_ratio).tolist()
            tau = (state[self.tau_idx]*self.cfg.current2Torque * self.cfg.gear_ratio).tolist()
            state_msg = FingerState(q=q, dq=dq, tau=tau)
            self.state_writer.write(state_msg)

    def forwardDDSCmds2UDP(self):
        cmd_msg = get_last_msg(self.cmd_reader, FingerCmd)
        if cmd_msg is not None:
            self.udp_cmd[self.q_des_idx] = cmd_msg.q
            self.udp_cmd[self.dq_des_idx] = cmd_msg.dq
            self.udp_cmd[self.kp_idx] = cmd_msg.kp
            self.udp_cmd[self.kd_idx] = cmd_msg.kv
            tau_ff = np.clip(cmd_msg.tau_ff, -self.cfg.max_torque, self.cfg.max_torque)
            current = (np.array(tau_ff)/self.cfg.gear_ratio)/self.cfg.current2Torque
            self.udp_cmd[self.tau_ff_idx] = current
            self.udp_bridge.sendCommand(self.udp_cmd.tolist())

class NYUFingerRobot:
    def __init__(self, robot_name):
        self.robot_name = robot_name
        self.state_topic_name = f'{robot_name}_robot_state'
        self.cmd_topic_name = f'{robot_name}_robot_cmd'
        self.participant = DomainParticipant()
        self.cmd_topic = Topic(self.participant,self.cmd_topic_name, FingerCmd)
        self.state_topic = Topic(self.participant,self.state_topic_name, FingerState)
        self.state_reader = DataReader(self.participant, self.state_topic)
        self.cmd_writer = DataWriter(self.participant, self.cmd_topic)
        self.state = None

    def setCommand(self, torque):
        assert torque.shape == (3,), 'The shape of the torque array should be (3,)'
        cmd = FingerCmd(tau_ff=torque.tolist(),
                        q = np.zeros_like(torque).tolist(),
                        dq = np.zeros_like(torque).tolist(),
                        kp = np.zeros_like(torque).tolist(),
                        kv = np.zeros_like(torque).tolist())
        self.cmd_writer.write(cmd)

    def getRobotState(self):
        state_msg = get_last_msg(self.state_reader, FingerState)
        if state_msg is not None:
            q = np.array(state_msg.q)
            dq = np.array(state_msg.dq)
            tau = np.array(state_msg.tau)
            return dict (q=q, dq=dq, tau=tau)
        else:
            return None
