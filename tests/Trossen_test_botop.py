import os
os.environ["XDG_SESSION_TYPE"] = "x11"
os.environ["WAYLAND_DISPLAY"] = ""

import robotic as ry
import numpy as np
import time

def move_synced(bot, C, q):
    bot.moveTo(q)
    while bot.getTimeToEnd()>0:
        bot.sync(C, .1)

def IK(C:ry.Config, pos):
    q0 = C.getJointState()
    komo = ry.KOMO(C, 1, 1, 0, False) #one phase one time slice problem, with 'delta_t=1', order=0
    komo.addObjective([], ry.FS.jointState, [], ry.OT.sos, [1e-1], q0) #cost: close to 'current state'
    komo.addObjective([], ry.FS.jointState, [], ry.OT.sos, [1e-1], qHome) #cost: close to qHome
    komo.addObjective([], ry.FS.positionDiff, ['follower_left_joint_5', 'target'], ry.OT.eq, [1e1]) #constraint: gripper position
    #komo.addObjective([], ry.FS.positionDiff, ['joint_5', 'target'], ry.OT.eq, [1e1]) #constraint: gripper position
    
    ret = ry.NLP_Solver(komo.nlp(), verbose=0) .solve()
    
    return [komo.getPath()[0], ret]

def parse_input(input:int, target_pos: np.ndarray, speed:float, increment=0.05) -> tuple[int, float]:
    #position change
    if input == ord('r'):
        target_pos[2] += increment
    elif input == ord('f'):
        target_pos[2] -= increment  
    elif input == ord('w'):
        target_pos[0] += increment
    elif input == ord('s'):
        target_pos[0] -= increment
    elif input == ord('a'):
        target_pos[1] -= increment
    elif input == ord('d'):
        target_pos[1] += increment
    elif input == ord('t'):
        increment = min(increment + 0.01, 0.1)
    elif input == ord('g'):
        increment = max(increment - 0.01, 0.01)
    elif input == ord('q'):
        return 1 , speed, increment
    else:
        print('Invalid input. Please enter one of the following: r, f, w, s, a, d, q')
    return 0 , speed, increment




if __name__ == '__main__':
    sim = False
    print('Trossen Test File')
    C = ry.Config()
    

    
    if sim:
        C.addFile('/home/shcherba/git/calibration/scenes/trossen/trossen.yml') 
        C.view(True) 
    else:
        C.addFile('/home/shcherba/git/calibration/scenes/trossen/trossen.yml')
     

    for i in C.getFrameNames():
        if C.getFrame(i).getJointType() != ry.JT.none:

            print(i, C.getFrame(i).getJointType() )


    C_dict = C.asDict()
    bot = ry.BotOp(C, useRealRobot=not sim)
    ry.params_print()
    max_speed = 5.0
    speed = 2.0
    increment = 0.1

    meas_frame = C.getFrame('cam_left_wrist')
    meas_frame.setShape(ry.ST.marker, [.1])

    #test move
    qHome = bot.get_qHome()
    q = bot.get_q()
    gripper_frame = C.getFrame('follower_left_joint_5')
    #gripper_frame = C.getFrame('joint_5')
    gripper_frame.setShape(ry.ST.marker, [.1])

    target_pos = gripper_frame.getPosition()
    target_frame = C.addFrame('target'). setShape(ry.ST.marker, [.1]) .setPosition(target_pos)

 
    while True:
        #user_input = input("Enter a command (+, -, w, s, a, d, q): ")
        bot.wait(C,forKeyPressed=True, forTimeToEnd=False)
        user_input = bot.getKeyPressed()
        exit, speed, increment = parse_input(user_input, target_pos, speed, increment)
        if exit:
            print("Exiting...")
            bot.home(C)
            break
        print(f"Target position: {target_pos}, speed: {speed}, increment: {increment}")
        target_frame.setPosition(target_pos)   
        target, ret = IK(C, target_pos)
        if ret.feasible:
            bot.moveTo(target, timeCost=speed, overwrite=True)
            while bot.getTimeToEnd()>0:
                if bot.getKeyPressed()==ord(' '):
                    print("cancelled")
                    bot.stop(C)
                    target_pos = gripper_frame.getPosition()
                    target_frame.setPosition(target_pos)
                    break
                bot.sync(C, .1)
            
        else:
            print('position not feasible')

    
    del C
    del bot
