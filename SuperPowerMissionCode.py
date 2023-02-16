# Copyright 2023 Shenendehowa CSD FLL team #3249 the CaptiBytes
# Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
# 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS “AS IS” AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


# The following code by Shenendehowa CSD FLL team #3249 the CaptiBytes is a python program designed to be run on a Lego Spike Prime robot. It completes the SuperPowered robot game missions from the 2022-2023 FLL season. This code can be found on GitHub in a repository owned by user ShenFLL.


from math import *
import hub
import uos
import utime


# ports object
ports = {
    "A": hub.port.A,
    "B": hub.port.B,
    "C": hub.port.C,
    "D": hub.port.D,
    "E": hub.port.E,
    "F": hub.port.F,
    "a": hub.port.A,
    "b": hub.port.B,
    "c": hub.port.C,
    "d": hub.port.D,
    "e": hub.port.E,
    "f": hub.port.F
}


# speed multiplier
speed = 1.025


# define running
running = False


# set drive motors
drive = ports["D"].motor.pair(ports["E"].motor)


# set attachment motors
attachment = ports["A"].motor.pair(ports["C"].motor)


# move tank function
# mt(distance in degrees,left motor speed,right motor speed)
def mt(amount,ls,rs,wait = True):
    drive.run_for_degrees(round(amount),speed_0=round(-ls*speed),speed_1=round(rs*speed))
    global running




    running = True
    def event(details):
        if details == drive.primary().EVENT_COMPLETED:
            global running
            running = False
    drive.callback(event)
    while running and wait:
        pass


# start tank function
# st(left motor speed,right motor speed)
def st(ls,rs):
    drive.run_at_speed(round(-ls*speed),round(rs*speed))


# stop tank function
def pt():
    drive.hold()


# reset drive function
def rt():
    drive.preset(0,0)


# reset attachments function
def ra():
    attachment.preset(0,0)


# move attachment function
# ma(port letter of motor,distance in degrees,motor speed)
def ma(ml,amount,ms,wait = True):
    ports[ml].motor.run_for_degrees(round(amount),speed=round(ms))
    if wait:
        mwt(ml)


# start attachment function
# sa(port letter of motor,motor speed)
def sa(ml,ms):
    ports[ml].motor.run_at_speed(round(ms))


# pause attachment function
def pa(ml):
    ports[ml].motor.hold()


# move attachments sync function
# mas(distance in degrees,left motor speed,right motor speed)
def mas(amount,ls,rs,wait = True):
    attachment.run_for_degrees(round(amount),speed_0=round(ls),speed_1=round(rs))
    global running
    running = True
    def event(details):
        if details == attachment.primary().EVENT_COMPLETED:
            global running
            running = False
    attachment.callback(event)
    while running and wait:
        pass


# get distance
def gd():
    return round((abs(drive.primary().get()[1]) + abs(drive.secondary().get()[1])) / 2)


# get reflected light function
# rl(port letter of light sensor)
def rl(ll):
    return ports[ll].device.get()[0]


# wait function
# wt(time in milliseconds)
def wt(ms):
    utime.sleep_ms(ms)


# motor wait function (internal plz don't touch)
def mwt(ml):
    while ports[ml].motor.busy(ports[ml].motor.BUSY_MOTOR):
        pass


# get gyro angle function
def ga():
    return hub.motion.yaw_pitch_roll()[0]


# reset gyro function (plz don't call with arguments, all arguments are internal)
def rg(ang = 0, yc = 0):
    hub.motion.yaw_pitch_roll(ang,yc)


# calibrate gyro function
def cg():
    rg()
    mt(250,25,25)
    mt(2560,-25,25)
    mt(500,-25,-25)
    rg(yc = ga() / 5)


# line follow function
# lf(port letter of light sensor,distance in degrees,speed,acceleration/gain,side of line)
def lf(ln,amount,ts,ac,side):
    ts *= speed
    x = 0
    t = utime.time()
    rt()


    while gd() < amount:
        if abs(70-rl(ln)) > 10:
            ac += 0.00001 * ts
        else:
            ac -= 0.0001 * ts
        if ac > 0.3:
            ac = 0.3
        if ac < 0:
            ac = 0
        if side == 0:
            st((rl(ln)-35)*ac+ts,(100-rl(ln))*ac+ts)
        else:
            st((100-rl(ln))*ac+ts,(rl(ln)-35)*ac+ts)
        x += 1
    pt()
    print(x/(utime.time()-t))


# gyro follow function
# gf(distance in degrees,speed,acceleration/gain,heading in degrees,direction forward (1) or backward (0))
def gf(amount,ts,ac,ang,dn):
    ts *= speed
    rt()
    while gd() < amount:
        left = (ang-ga())*10
        right = (ga()-ang)*10
        if dn == 1:
            st(left*ac+ts,right*ac+ts)
        else:
            st(right*-ac-ts,left*-ac-ts)
    pt()


# gyro turn function
# gt(angle to turn to)
def gt(ang):
    rt()
    for i in range(5):
        while not(ga()<ang+(10-i) and ga()>ang-(10-i)):
            if ang-ga()>0:
                st((ang-ga()+70)/(i+10),(ga()-ang-70)/(i+10))
            else:
                st((ang-ga()-70)/(i+10),(ga()-ang+70)/(i+10))
    pt()


# play sound function
# sound(index of sound file in the sounds folder)
def sound(sn):
    hub.sound.play("/sounds/"+uos.listdir("/sounds")[sn])


# set hub led color function
# led(integer that corresponds to some color)
def led(n):
    hub.led(n)


# exit function
def exit():
    raise SystemExit


# get temperature function
def temp():
    return hub.temperature()*9/5+32


# purple mission function
def purple(imaquitter):
    led(2)
    if imaquitter:
        return
    rg()
    gf(980,45,0.1,0,1)
    gf(200,60,0.1,0,0)
    gt(-40)
    gf(960,40,0.05,-40,1)
    gt(45)
    gf(800,30,0.05,45,1)
    gf(200,50,0.05,45,0)
    for i in range(3):
        gf(450,40,0.05,45,1)
        wt(300)
        gf(200,40,0.05,45,0)
    gf(510,30,0.05,48,0)
    gf(30,30,0.05,45,1)
    gt(-55)
    ma("C",400,-100)
    wt(300)
    gf(500,25,0.1,-55,1)
    gt(-30)
    ma("C",350,50)
    ma("C",300,-50)
    gt(-45)
    mt(2000,-70,-70)
    ma("C",500,100)


# orange mission function
def orange(imaquitter):
    led(8)
    if imaquitter:
        return
    rg()
    gf(1200,60,0.1,0,1)
    gf(200,20,0.1,0,1)
    ma("A",300,-100)
    ma("C",350,-100)
    wt(200)
    gf(1400,-70,0.1,0,1)


# blue mission function
def blue(imaquitter):
    led(3)
    if imaquitter:
        return
    rg()
    ma("A",350,-20, wait = False)
    gf(150,30,0.1,0,1)
    gt(-70)
    gf(1500,50,0.1,-70,1)
    gt(0)
    gf(850,50,0.1,0,1)
    gf(550,-50,0.1,0,1)
    gt(-40)
    gf(800,30,0.1,-41,1)
    gt(-15)
    gf(200,30,0.1,-15,1)
    gt(-90)
    gf(350,30,0.1,-90,1)
    gt(-145)
    gf(1700,65,0.1,-145,1)


# pink mission function
def pink(imaquitter):
    led(1)
    if imaquitter:
        return
    rg()
    gf(700,25,0.1,0,1)
    ma("A",150,100)
    wt(200)
    ma("C",350,100)
    ma("A",80,-100)
    gf(900,-70,0.1,0,1)


# yellow mission function
def yellow(imaquitter):
    led(7)
    if imaquitter:
        return
    rg()
    gf(800,35,0.08,0,1)
    gt(45)
    gf(400,35,0.08,45,1)
    gt(0)
    gf(500,35,0.08,0,1)
    for i in range(3):
        ma("A",500,-100)
        ma("A",500,100)
    gf(500,75,0.08,0,0)
    gt(45)
    gf(400,75,0.08,45,0)
    gt(0)
    gf(800,75,0.08,0,0)


# green mission function
def green(imaquitter):
    led(5)
    if imaquitter:
        return
    rg()
    gf(100,30,0.1,0,1)
    gt(65)
    wt(200)
    gf(1340,40,0.1,65,1)
    gt(-40)
    wt(200)
    gf(200,35,0.1,-40,1)
    gf(50,-20,0.1,-40,1)
    ma("A",200,100)
    gt(0)
    ma("C",90,10)
    gf(300,-50,0.1,0,1)
    gt(90)
    gf(2000,70,0.1,90,1)


# red mission function
def red(imaquitter):
    led(9)
    if imaquitter:
        return
    rg()
    mt(1000,40,40)
    mt(200,-40,-40)


numbers = [hub.Image("""00900
09900
90900
00900
99999
"""),hub.Image("""99990
00009
00990
09000
99999
"""),hub.Image("""99990
00009
99990
00009
99990
"""),hub.Image("""09090
90090
99999
00090
00090
"""),hub.Image("""99999
90000
99990
00009
99990
"""),hub.Image("""09999
90000
99990
90009
09990
"""),hub.Image("""99999
00090
00900
09000
90000
"""),hub.Image("""09990
90009
09990
90009
09990
"""),hub.Image("""099990
90009
09999
00009
00009
""")]


# list containing order of missions
programs = [purple,orange,blue,pink,yellow,green,red]


# main function manages inputs, display, and running programs 
def main():
    program = 0
    while program < len(programs):
        hub.display.show(numbers[program])
        programs[program](True)
        while not(hub.button.connect.is_pressed() or hub.button.left.is_pressed() or hub.button.right.is_pressed()):
            pass
        if hub.button.left.is_pressed():
            wt(100)
            if hub.button.right.is_pressed():
                wt(2000)
                cg()
                wt(2000)
            else:
                program -= 1
                while hub.button.left.is_pressed():
                    pass
        elif hub.button.right.is_pressed():
            wt(100)
            if hub.button.left.is_pressed():
                wt(2000)
                cg()
                wt(2000)
            else:
                program += 1
                while hub.button.right.is_pressed():
                    pass
        else:
            wt(500)
            programs[program](False)
            program += 1
    exit()




# run everything
main()