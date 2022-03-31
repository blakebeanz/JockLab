from machine import Pin, PWM, UART
import time, utime
#init motionSensor
trigger = Pin(27, Pin.OUT)
echo = Pin(26, Pin.IN, Pin.PULL_UP)
SSDistance = 0
SSDistancePrev = 0
SSCounter1 = 0
SSCounter2 = 0
SSSafe = False
CyclesWithoutSS = 0
def ultra():
    trigger.value(0)
    utime.sleep_us(2)
    trigger.value(1)
    utime.sleep_us(5)
    trigger.value(0)
    """
    while echo.value() == 0:
        signaloff = utime.ticks_us()
    while echo.value() == 1:
        signalon = utime.ticks_us()
    timepassed = signalon - signaloff
    distanceSS = (timepassed * 0.0343) / 2
    return distanceSS
    """
EchoStart = 0
def EchoRiseHandler(sens):
    global EchoStart
    #print('Rising')
    EchoStart = utime.ticks_us()
    echo.irq(EchoFallHandler,Pin.IRQ_FALLING)
def EchoFallHandler(sens):
    global EchoStart, EchoFall,SSDistance
    #print('Falling')
    SSDistance = ((utime.ticks_us() - EchoStart)*0.0343)/2
    echo.irq(EchoRiseHandler,Pin.IRQ_RISING)
    ultra()
echo.irq(EchoRiseHandler,Pin.IRQ_RISING)
#end motionsensor
#init PWM
#Left Motor
PwmLeft = PWM(Pin(21))
PwmLeft.freq(200)
PwmLeft.duty_ns(1500000) #idle throttle
print('Left Initialized')

led = Pin(25,Pin.OUT)
#Right Motor
PwmRight = PWM(Pin(20))
PwmRight.freq(200)
PwmRight.duty_ns(1500000) #idle throttle
print('Right Initialized')

# MaxForward = 1900000 ns pulse
# MaxReverse = 1100000 ns pulse
TireDiameter = 4
MPHLeft = 0
MPHRight = 0
MPHRightPrev = 0
MPHLeftPrev = 0
TargetVelocity = 0
SignalToMotorLeft = 1500000
SignalToMotorRight = 1500000
errorRightPrev = 0
errorLeftPrev = 0
TargetVelocityPrev = 0
#init encoder and its interrupt
#Left Encoder
EncoderLeft = Pin(19,Pin.IN, Pin.PULL_UP)
EncoderCountsLeft = 0
EncoderCountsLeftPrev = 0
StartRotationLeft = 0
ProcedureState = 0
ReverseMode = False
def EncoderHandlerLeft(enc):
    global EncoderCountsLeft,StartRotationLeft,MPHLeft
    EncoderCountsLeft += 1
    #print(EncoderCounts)
    if (EncoderCountsLeft % 9 == 0):
        #EncoderCountsLeft = 0
        RotationTime = time.ticks_us() - StartRotationLeft
        StartRotationLeft = time.ticks_us()
        RPM = 0.25 / (RotationTime/60000000)
        MPHLeft = (RPM*TireDiameter*3.141592653*60)/63360
        #print('Left: {}'.format(MPHLeft))
"""
def EncoderHandlerLeft(enc):
    global EncoderCountsLeft
    EncoderCountsLeft += 1
"""
#Right Encoder
EncoderRight = Pin(18,Pin.IN, Pin.PULL_UP)
EncoderCountsRight = 0
EncoderCountsRightPrev = 0
StartRotationRight = 0
def EncoderHandlerRight(enc):
    global EncoderCountsRight,StartRotationRight,MPHRight
    EncoderCountsRight += 1
    #print(EncoderCounts)
    if (EncoderCountsRight % 9 == 0):
        #EncoderCountsRight = 0
        RotationTime = time.ticks_us() - StartRotationRight
        StartRotationRight = time.ticks_us()
        RPM = 0.25 / (RotationTime/60000000)
        MPHRight = (RPM*TireDiameter*3.141592653*60)/63360
        #print('Right: {}'.format(MPHRight))
"""
def EncoderHandlerRight(enc):
    global EncoderCountsRight
    EncoderCountsRight += 1
"""
EncoderLeft.irq(EncoderHandlerLeft,Pin.IRQ_RISING)
EncoderRight.irq(EncoderHandlerRight,Pin.IRQ_RISING)

def Reverse():
    global PwmLeft,PwmRight,ReverseMode
    PwmLeft.duty_ns(1430000)
    PwmRight.duty_ns(1430000)
    time.sleep(1)
    PwmLeft.duty_ns(1500000)
    PwmRight.duty_ns(1500000)
    ReverseMode=True
def Forward():
    global PwmLeft,PwmRight,ReverseMode
    PwmLeft.duty_ns(1520000)
    PwmRight.duty_ns(1520000)
    time.sleep(0.05)
    PwmLeft.duty_ns(1500000)
    PwmRight.duty_ns(1500000)
    ReverseMode = False
    
    

#init UART
uart = UART(0,9600, bits=8, parity=None, stop=1, tx = Pin(16), rx = Pin(17)) # init with given parameters
#end init UART

SignalToMotor = 1500000
signal = ''
Decoded = ''
Cycles = 0
#CF = 0
#timer before start
for i in range(3,0,-1):
    time.sleep(1)
    print('Starting in {}'.format(i))
#TestVarsToDelete

#ENDTESTVARS
#Start Motion Sensor
ultra()


####################################RuntimeLoop################################
while True:
    start = time.ticks_ms()
#user code goes here
#check proximity sensor
    if SSDistance < 100:
        SSCounter1 += 1
        SSCounter2 = 0
        if SSCounter1>5:
            led.value(1)
            SSSafe = False
    else :
        SSCounter2 += 1
        
    if SSCounter2 > 50 :
        SSCounter1 = 0
        led.value(0)
        SSafe = True
    
    if SSDistance != SSDistancePrev:
        print(SSDistance)
        CyclesWithoutSS = 0
    else :
        CyclesWithoutSS += 1
    if CyclesWithoutSS > 200:
        CyclesWithoutSS = 0
        ultra()
        print('reboot SS attempt')
    SSDistancePrev = SSDistance
#check acceleromater
    DistanceRight = EncoderCountsRight*0.0291 #distance in ft
    DistanceLeft = EncoderCountsLeft*0.0291 #distance in ft
#print(Distance)
    signal = uart.read()
    if signal:
        print('Signal is : {}'.format(signal))
        print(chr(signal[0]))
        if signal[0] in range(48,57):
            Cycles = int(chr(signal[0]))

    
#Start Accelerate and decellerate Run
    if Cycles > 0:
        if ProcedureState == 0:
            TargetVelocity = 3 # starts motors
            TargetDistance = 5 # sets distance
            ProcedureState = 1 # next step
            EncoderCountsRight = 0 # reset encoder counts
            EncoderCountsLeft = 0
            
        elif ProcedureState == 1:
            if (DistanceLeft >= TargetDistance) and (DistanceRight >= TargetDistance):
                Reverse()
                #Brake algorithm here
                TargetVelocity = 0 # set velocity to 0
                ProcedureState = 2 # reached distance
                print('Reached Distance')
                
        elif ProcedureState == 2:
            #Reverse() # enter reverse mode
            TargetVelocity = -3
            EncoderCountsRight = 0
            EncoderCountsLeft = 0
            MPHRight = 0
            MPHLeft = 0
            ProcedureState = 3
            SignalToMotorLeft = 1470000
            SignalToMotorRight = 1470000
            print('PS 2')
            
        elif ProcedureState == 3:
            #print('Right : {} ,Left : {}'.format(SignalToMotorRight,SignalToMotorLeft))
            if (DistanceLeft >= TargetDistance) and (DistanceRight >= TargetDistance):
                Forward()
                #Reverse Brake algorithm here
                Cycles -= 1
                ProcedureState = 0
                TargetVelocity = 0
                print('Initial Position')            
                  
            
        
#PID start
    if TargetVelocity != TargetVelocityPrev:
        print('Target Velocity : {}'.format(TargetVelocity))
        TargetVelocityPrev = TargetVelocity
    
    if ReverseMode:
        errorRight = TargetVelocity + MPHRight
        #print('ErrorRight {}'.format(errorRight))
        errorLeft = TargetVelocity + MPHLeft
        #print('ErrorLeft {}'.format(errorLeft))
    else:
        errorRight = TargetVelocity - MPHRight
        #print('ErrorRight {}'.format(errorRight))
        errorLeft = TargetVelocity - MPHLeft
        #print('ErrorLeft {}'.format(errorLeft))

    if MPHRight < abs(TargetVelocity):
        SignalToMotorRight += int(errorRight*50 + errorRightPrev*24)
        #print('Right+')
    if MPHLeft < abs(TargetVelocity):
        SignalToMotorLeft += int(errorLeft*52 + errorLeftPrev*27)
        #print('Left+')
    if TargetVelocity == 0 :
        SignalToMotorRight = 1500000
        SignalToMotorLeft = 1500000
        MPHRight = 0
        MPHLeft = 0
    errorRightPrev = errorRight
    errorLeftPrev = errorLeft
#PID end
    
    
    PwmRight.duty_ns(SignalToMotorRight)
    PwmLeft.duty_ns(SignalToMotorLeft)
    if ((MPHRight != MPHRightPrev) or (MPHLeft != MPHLeftPrev)):
        print('Left {}, Right {} PWMLeft {}, PWMRight{}'.format(MPHLeft,MPHRight,PwmRight.duty_ns(),PwmLeft.duty_ns()))
    MPHRightPrev = MPHRight
    MPHLeftPrev = MPHLeft
    #print('Encoders L: {} R: {}'.format(EncoderCountsLeft, EncoderCountsRight))
#end code
    #print('time elapsed = {}'.format(time.ticks_ms()-start))
    while (time.ticks_ms() < start + 10):
        pass

