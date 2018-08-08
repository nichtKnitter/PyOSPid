"""
# Port of the OS-PID Library to Python by Max Baumgartner, baumax@posteo.de
# Original header File:
# /**********************************************************************************************
#  * Arduino PID Library - Version 1.0.1
#  * by Brett Beauregard <br3ttb@gmail.com> brettbeauregard.com
#  *
#  * This Library is licensed under a GPLv3 License
#  **********************************************************************************************/
"""

import time

"""
/*Constructor (...)*********************************************************
* The parameters specified here are those for for which we can't set up
* reliable defaults, so we need to have the user set them.
***************************************************************************/
PID::PID(double* Input, double* Output, double* Setpoint,
        double Kp, double Ki, double Kd, int ControllerDirection)
{
PID::SetOutputLimits(0, 255); //default output limit corresponds to
//the arduino pwm limits

    SampleTime = 100; //default Controller Sample Time is 0.1 seconds

    PID::SetControllerDirection(ControllerDirection);
    PID::SetTunings(Kp, Ki, Kd);

    lastTime = millis()-SampleTime;
    inAuto = false;
    myOutput = Output;
    myInput = Input;
    mySetpoint = Setpoint;

}
"""


class OsPID:
    def __init__(self, startInput, startOutput, Setpoint, Kp, Ki, Kd, ControllerDirection="DIRECT", inAuto=False):

        self.isInAutomaticMode = inAuto

        self.controllerDirection = ControllerDirection
        if ControllerDirection == "DIRECT":
            self.isDirectAndNotReverse = True

        self.isInAutomaticMode = inAuto

        self.sampletime = 0.1  # default Controller Sample Time is 0.1 seconds
        self.now = time.time()
        self.lastTime = time.time() - self.sampletime

        self.myOutput = startOutput
        self.input = startInput
        self.setpoint = Setpoint

        self.ITerm = 0
        self.lastInput = startInput

        self.setOutputLimits(0, 100)  # regelt zwischen 0 und 100 %
        self.SetControllerDirection(ControllerDirection)
        self.SetTunings(Kp, Ki, Kd)

    def setSetpoint(self, newSetpoint):
        self.setpoint = newSetpoint

    def compute(self, Input):
        """
        /* Compute() **********************************************************************
        * This, as they say, is where the magic happens. this function should be called
        * every time "void loop()" executes. the function will decide for itself whether a new
        * pid Output needs to be computed
        **********************************************************************************/
        """
        if self.isInAutomaticMode is False:
            return
        now = time.time()
        timeChange = (now - self.lastTime)
        if timeChange >= self.sampletime:
            error = self.setpoint - Input
            print("setpoint", self.setpoint, "input", Input)
            print("error ", error)

            # Postitional algorithm
            self.ITerm += (self.Ki * error)
            if self.ITerm >= self.outMax:
                self.ITerm = self.outMax
            elif self.ITerm < self.outMin:
                self.ITerm = self.outMin
            self.dInput = (Input - self.lastInput)

            #       /*Compute PID Output*/
            output = self.Kp * error + self.ITerm - self.Kd * self.dInput

            # Alternative velocity PID:
            # output = lastOutput + Kp*(error - lastError) + Ki * error

            if output > self.outMax:
                output = self.outMax
            elif (output < self.outMin):
                output = self.outMin
            self.lastInput = Input
            self.lastTime = now

            self.myOutput = output
            print("Stellgrad: ", output)
            return output

    def SetTunings(self, Kp, Ki, Kd):
        if Kp < 0 or Ki < 0 or Kd < 0:
            return
        self.Kp = Kp
        self.Ki = Ki * self.sampletime
        self.Kd = Kd / self.sampletime
        if self.controllerDirection == "REVERSE":
            self.Kp = 0 - Kp
            self.Ki = 0 - Ki
            self.Kd = 0 - Kd
        return self.Kp, self.Ki, self.Ki

    def setSampleTime(self, NewSampleTime):
        """
        /* SetSampleTime(...) *********************************************************
        * sets the period, in seconds, at which the calculation is performed
        ******************************************************************************/
        """
        if (NewSampleTime > 0):
            ratio = NewSampleTime / self.sampletime
            self.Ki *= ratio
            self.Kd /= ratio
            self.sampletime = NewSampleTime

    def setOutputLimits(self, Min=0, Max=100):
        """
         /* SetOutputLimits(...)****************************************************
        * This function will be used far more often than SetInputLimits. while
        * the input to the controller will generally be in the 0-1023 range (which is
        * the default already,) the output will be a little different. maybe they'll
        * be doing a time window and will need 0-8000 or something. or maybe they'll
        * want to clamp it from 0-125. who knows. at any rate, that can all be done
        * here.
        **************************************************************************/
        :param outMin:
        :param outMax:
        :return:
        """

        if Min >= Max:
            return
        self.outMin = Min
        self.outMax = Max
        if self.isInAutomaticMode:
            if self.myOutput > self.outMax:
                self.myOutput = self.outMax
            elif self.myOutput < self.outMin:
                self.myOutput = self.outMin

            if self.ITerm > self.outMax:
                self.ITerm = self.outMax
            elif self.ITerm < self.outMin:
                self.ITerm = self.outMin

    def setMode(self, newMode="Automatic"):
        """
        /* SetMode(...)****************************************************************
        * Allows the controller Mode to be set to manual (0) or Automatic (non-zero)
        * when the transition from manual to auto occurs, the controller is
        * automatically initialized
        ******************************************************************************/
        :param Mode:
        :return:
        """
        if newMode == "Automatic":
            isModeAutomatic = True
        else:
            isModeAutomatic = False

        newAuto = (isModeAutomatic is

            True)
        if newAuto != self.isInAutomaticMode:
            # { /*we just went from manual to auto*/
            self.PIDinit()
        self.isInAutomaticMode = newAuto

    def PIDinit(self):
        """
        /* Initialize()****************************************************************
        * does all the things that need to happen to ensure a bumpless transfer
        * from manual to automatic mode.
        ******************************************************************************/
        :return:
        """
        self.ITerm = self.myOutput
        self.lastInput = self.input
        if self.ITerm > self.outMax:
            self.ITerm = self.outMax
        elif self.ITerm < self.outMin:
            self.ITerm = self.outMin

    def SetControllerDirection(self, newDirection="DIRECT"):
        """
        * The PID will either be connected to a DIRECT acting process (+Output leads
        * to +Input) or a REVERSE acting process(+Output leads to -Input.) we need to
        * know which one, because otherwise we may increase the output when we should
        * be decreasing. This is called from the constructor.
        :return:
        """
        if newDirection == "DIRECT":
            newBoolControllerDirection = True
        else:
            newBoolControllerDirection = False
        if self.isInAutomaticMode and newBoolControllerDirection != self.isDirectAndNotReverse:
            self.Kp = 0 - self.Kp
            self.Ki = 0 - self.Ki
            self.Kd = 0 - self.Kd
