from FingerPy.robot.interface import TeensyUDPBridge, RobotConfig, UDPCycloneDDSBridge
import signal
import time

config = RobotConfig
config.DoF = 3
config.robot_name = 'finger1'
config.state_fields = [
                       'q_hip',   'q_thigh',   'q_knee',
                       'dq_hip',  'dq_thigh',  'dq_knee', 
                       'tau_hip', 'tau_thigh', 'tau_knee'
                      ]

config.command_fields = [
                         'tau_ff_hip', 'tau_ff_thigh',  'tau_ff_knee',
                         'q_des_hip',  'q_des_thigh',   'q_des_knee', 
                         'dq_des_hip', 'dq_des_thigh',  'dq_des_knee', 
                         'kp_hip',     'kp_thigh',      'kp_knee',
                         'kd_hip',     'kd_thigh',      'kd_knee'
                         ]

config.current2Torque = 1.0
config.gear_ratio = 9
config.max_torque = 20.0
config.robot_ip  = '192.168.123.10'
config.robot_port = 5000
config.local_port = 5000
config.dds_freq = 1000

udp_bridge = TeensyUDPBridge(config)
dds_bridge = UDPCycloneDDSBridge(udp_bridge, config)

def signal_handler(sig, frame):
    print('Terminating the bridge ...')
    udp_bridge.close()
    exit(0)

signal.signal(signal.SIGINT, signal_handler)

def main():
    print('Bridge is running ...')
    while True: 
        start=time.time()
        dds_bridge.forwardUDPStates2DDS()
        dds_bridge.forwardDDSCmds2UDP()
        while time.time()-start<(1./config.dds_freq):
            time.sleep(0.0002)

if __name__ == '__main__':
    main()