

def PID_Asli (Error,Speed,Voltage,t, Kp, Ki, Kd, Sp, step_size):
    Er = (Sp - Speed[t]) #calcualte the error btw setpoint and speed

    #detirmine the new control position (Voltage)
    control = Voltage[t] + (Kp*Er + Ki*(Er + Error[t])/2 + \
                Kd*(Er-Error[t]))*step_size

    return control,Er


def PID_ES (Error,Speed,Voltage,t, Kp_es, Ki_es, Kd_es, Sp, step_size):
    Er = (Sp - Speed[t]) #calcualte the error btw setpoint and speed

    #detirmine the new control position (Voltage)
    control = Voltage[t] + (Kp_es*Er + Ki_es*(Er + Error[t])/2 + \
                Kd_es*(Er-Error[t]))*step_size

    return control, Er

def PID_PSO (Error,Speed,Voltage,t, Kp_pso, Ki_pso, Kd_pso, Sp, step_size):
    Er = (Sp - Speed[t]) #calcualte the error btw setpoint and speed

    #detirmine the new control position (Voltage)
    control = Voltage[t] + (Kp_pso*Er + Ki_pso*(Er + Error[t])/2 + \
                Kd_pso*(Er-Error[t]))*step_size

    return control,Er


def PID_EVO_PSO (Error,Speed,Voltage,t, Kp_evopso, Ki_evopso, Kd_evopso, Sp, step_size):
    Er = (Sp - Speed[t]) #calcualte the error btw setpoint and speed

    #detirmine the new control position (Voltage)
    control = Voltage[t] + (Kp_evopso*Er + Ki_evopso*(Er + Error[t])/2 + \
                Kd_evopso*(Er-Error[t]))*step_size

    return control,Er



class controller(object):
    def __init__ (self):
        self.Sp = 1               #angular velocity setpoint
        self.step_size = .01      #seconds
		
		#PID Controller
        #self.Kp = 1               #proportional control
        #self.Ki = 0               #integral control
        #self.Kd = 10              #derivative control
        """
		#PID ES
        self.Kp_es = best_pid_evo.get('Kp')
        self.Ki_es = best_pid_evo.get('Ki')
        self.Kd_es = best_pid_evo.get('Kd')
        
		#PID PSO
        self.Kp_pso = best_pid_pso.get('Kp')
        self.Ki_pso = best_pid_pso.get('Ki')
        self.Kd_pso = best_pid_pso.get('Kd')
        
		#PID EVO-PSO
        self.Kp_evopso = best_pid_evo_pso.get('Kp')
        self.Ki_evopso = best_pid_evo_pso.get('Ki')
        self.Kd_evopso = best_pid_evo_pso.get('Kd')

       """

    #create a function that returns the PID control variable

    """
    #create a function that returns the PID_ES control variable


    #create a function that returns the PID_PSO control variable
 

    #create a function that returns the PID_EVO_PSO control variable

"""


"""
#############################################################################
#-----------------Run episode under PID_ES control-------------------------
##############################################################################
Speed_ES = [StPt]         #initialize a vector to keep track of angular velocity
Voltage_ES = [StPt]        #initalize a vector to keep track of Voltage
Error_ES = [StPt]            #initalize a vector to keep track of the error term 
SetPoint_ES = [StPt]        #initalize a vector to keep track of the setpoint term 

for t in range(episodes):
    
    #get control move
    V_ES,Er_ES = Control.PID_ES(Error_ES,Speed_ES,Voltage_ES,t)

    #advacnce motor model by one time step and get new angular velocity
    w_ES = motor.step(Speed_ES[t],V_ES,Control.step_size)

    #append voltage,speed, and error history to their vectors
    Speed_ES = np.append(Speed_ES, w_ES)
    Voltage_ES = np.append(Voltage_ES, V_ES)
    Error_ES = np.append(Error_ES, Er_ES)
    SetPoint_ES = np.append(SetPoint_ES, Control.Sp)

###########################################################################
#-----------------Run episode under PID_PSO control----------------------------
############################################################################
Speed_PSO = [StPt]         #initialize a vector to keep track of angular velocity
Voltage_PSO = [StPt]        #initalize a vector to keep track of Voltage
Error_PSO = [StPt]            #initalize a vector to keep track of the error term 
SetPoint_PSO = [StPt]        #initalize a vector to keep track of the setpoint term 


for t in range(episodes):
    
    #get control move
    V_PSO,Er_PSO = Control.PID_PSO(Error_PSO,Speed_PSO,Voltage_PSO,t)

    #advacnce motor model by one time step and get new angular velocity
    w_PSO = motor.step(Speed_PSO[t],V_PSO,Control.step_size)

    #append voltage,speed, and error history to their vectors
    Speed_PSO = np.append(Speed_PSO, w_PSO)
    Voltage_PSO = np.append(Voltage_PSO, V_PSO)
    Error_PSO = np.append(Error_PSO, Er_PSO)
    SetPoint_PSO = np.append(SetPoint_PSO, Control.Sp)


###########################################################################
#-----------------Run episode under PID_ES+PSO control--------------------
###########################################################################
Speed_EVO_PSO = [StPt]         #initialize a vector to keep track of angular velocity
Voltage_EVO_PSO = [StPt]        #initalize a vector to keep track of Voltage
Error_EVO_PSO = [StPt]            #initalize a vector to keep track of the error term 
SetPoint_EVO_PSO = [StPt]        #initalize a vector to keep track of the setpoint term 


for t in range(episodes):
    
    #get control move
    V_EVO_PSO,Er_EVO_PSO = Control.PID_EVO_PSO(Error_EVO_PSO,Speed_EVO_PSO,Voltage_EVO_PSO,t)

    #advacnce motor model by one time step and get new angular velocity
    w_EVO_PSO = motor.step(Speed_EVO_PSO[t],V_EVO_PSO,Control.step_size)

    #append voltage,speed, and error history to their vectors
    Speed_EVO_PSO = np.append(Speed_EVO_PSO, w_EVO_PSO)
    Voltage_EVO_PSO = np.append(Voltage_EVO_PSO, V_EVO_PSO)
    Error_EVO_PSO = np.append(Error_EVO_PSO, Er_EVO_PSO)
    SetPoint_EVO_PSO = np.append(SetPoint_EVO_PSO, Control.Sp)
    
#########################################################################
#-----------------------Results-----------------------------------------
########################################################################
"""

#calculate Error Sum metircs
#SumE_PID = np.sum(abs(Speed - SetPoint))*Control.step_size
#SumE_PID_ES = np.sum(abs(Speed_ES - SetPoint_ES))*Control.step_size
#SumE_PID_PSO = np.sum(abs(Speed_PSO - SetPoint_PSO))*Control.step_size
#SumE_PID_EVO_PSO = np.sum(abs(Speed_EVO_PSO - SetPoint_EVO_PSO))*Control.step_size

#reversal amplitude
#Max_PID = np.max(Voltage)
#Max_PID_ES = np.max(Voltage_ES)
#Max_PID_PSO = np.max(Voltage_PSO)
#Max_EVO_PSO = np.max(Voltage_EVO_PSO)


#PID_count = reversal_count(Voltage)
#PID_ES_count = reversal_count(Voltage_ES)
#PID_PSO_count = reversal_count(Voltage_PSO)
#PID_EVO_PSO_count = reversal_count(Voltage_EVO_PSO)

