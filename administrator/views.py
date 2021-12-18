from django.shortcuts import render, redirect
from django.conf import settings
from django.contrib.auth.decorators import login_required
from django.utils.decorators import decorator_from_middleware
from django.core.files.storage import default_storage
from django.core.files.base import ContentFile
from django.contrib import messages
#from django_globals import globals
import random
from operator import itemgetter
import zipfile
import os
from shutil import copyfile

#Plotly
import plotly.express as px
import plotly.figure_factory as ff
from plotly.offline import plot
import plotly.graph_objects as go


#End Plotly

import pandas as pd
import numpy as np
from time import time

#Sklearn
from sklearn.metrics import accuracy_score
from sklearn.preprocessing import LabelEncoder
from sklearn.model_selection import train_test_split, RepeatedStratifiedKFold
from sklearn import naive_bayes
from sklearn.metrics import confusion_matrix, classification_report, accuracy_score
from genetic_selection import GeneticSelectionCV

#numpy
import numpy as np

#Hyperactive
from hyperactive import *

#Controller PID
from administrator.controller_PID import *

Sp = 1
step_size = .01



class DCmotor(object):
    def __init__(self,J,R,K):
      self.J = J
      self.R = R
      self.K = K
    
    def step (self,w,V,stepsize):
      Ts = self.K*V/self.R
      wf = V/ self.R
      dwdt = (Ts/self.J) * (1-w/wf)
      dw = dwdt*stepsize
      new_w = w+dw
      return new_w

#SETPOINT

def reversal_count(Voltage, episodes):
    slope = True
    count = 0
    for i in range(1, episodes):
        if Voltage[i]-Voltage[i-1] >= 0:
            new_slope = True
        else: new_slope = False
        
        if new_slope != slope:
            count += 1
        
        slope = new_slope
    
    return count

def step_info(yout,t):

    Os = (yout.max()/yout[-1]-1)*100
    Tr = t[next(i for i in range(0,len(yout)-1) if yout[i]>yout[-1]*.90)]-t[0]
    Ts = t[next((len(yout)-i for i in range(2,len(yout)-1) if abs(yout[-i]/yout[-1])>1.02), 0)]-t[0]
    #print("OS: %f%s"%((yout.max()/yout[-1]-1)*100,'%'))
    #print("Tr: %fs"%(t[next(i for i in range(0,len(yout)-1) if yout[i]>yout[-1]*.90)]-t[0]))
    #print("Ts: %fs"%(t[next(len(yout)-i for i in range(2,len(yout)-1) if abs(yout[-i]/yout[-1])>1.02)]-t[0]))

    return Os, Tr, Ts

J = .01
K = .01
R = 1 
episodes = 1000

best_pid_evo = None
best_pid_pso = None
best_pid_evo_pso = None
best_es_var = None
pid_evo_pso_score = -100
SetPoint = None
pid_val = None
hasil1 = None
# Create your views here.


@login_required(login_url=settings.LOGIN_URL)
def index(request):
    return render(request, 'administrator/dashboard.html')

@login_required(login_url=settings.LOGIN_URL)
def tentang(request):
    return render(request, 'administrator/tentang.html')

#########################################
#       Inisialisasi Paramater PID      #
#########################################

@login_required(login_url=settings.LOGIN_URL)
def ParamPID(request):
    
    if request.method == 'POST':
        global J
        global K
        global R
        global episodes
        
        J = float(request.POST['J'])
        K = float(request.POST['K'])
        R = float(request.POST['R'])
        episodes = int(request.POST['episodes'])
    return render(request, 'administrator/ParamPID.html',)

#########################################
#                  PID                  #
#########################################

@login_required(login_url=settings.LOGIN_URL)
def PID(request):
    return render(request, 'administrator/PID.html')


##Main Module

@login_required(login_url=settings.LOGIN_URL)
def HasilPID(request):
    
    global pid_val

    if request.method == 'GET':
        
        Kp = int(request.GET['Kp'])
        Ki = int(request.GET['Ki'])
        Kd = int(request.GET['Kd'])
        
        pid_val = {
            "Kp": Kp,
            "Ki": Ki,
            "Kd": Kd,
        }
             
        motor = DCmotor(J,R,K)
        StPt = 0
    ##############################################################################
    #-----------------------------#Run episode under PID control------------------
    ############################################################################

        Speed = [StPt]         #initialize a vector to keep track of angular velocity
        Voltage = [StPt]        #initalize a vector to keep track of Voltage
        Error = [StPt]            #initalize a vector to keep track of the error term 
        SetPoint = [StPt]        #initalize a vector to keep track of the setpoint term 

        for t in range(episodes):

            #get control move
            V,Er = PID_Asli(Error, Speed, Voltage, t, Kp, Ki, Kd, Sp, step_size)

            #advacnce motor model by one time step and get new angular velocity
            w = motor.step(Speed[t],V,step_size)

            #append voltage,speed, and error history to their vectors
            Speed = np.append(Speed,w)
            Voltage = np.append(Voltage,V)
            Error = np.append(Error,Er)
            SetPoint = np.append(SetPoint,Sp)
            
        SumE_PID = np.sum(abs(Speed - SetPoint))*step_size
        Max_PID = np.max(Voltage)
        PID_count = reversal_count(Voltage, episodes)
        
        #print(SumE_PID)
        #print(Max_PID)
        #print(PID_count)
        
        fig_report = go.Figure()
        fig_report.add_trace(go.Scatter(x=list(range(1, episodes)),
                                        y = SetPoint,
                                        mode='lines',
                                        name='SetPoint'
                                        ))
        
        fig_report.add_trace(go.Scatter(x=list(range(1, episodes)),
                                        y = Speed,
                                        mode='lines',
                                        name='PID'
                                        ))
        
        fig_report.update_layout(
                            title="System Response",
                            xaxis_title="Time (.01s)",
                            yaxis_title="Angular Velocity",
                            #legend_title="Method",
                            font=dict(
                                family="Courier New, monospace",
                                size=18,
                                color="RebeccaPurple"
                            )
                        )
        
        fig_PID = plot(fig_report, output_type='div')
        
                   
        return render(request, 'administrator/HasilPID.html', context={'plot_PID': fig_PID,
                                                                       'Error' : SumE_PID,
                                                                       'Max_PID':Max_PID,
                                                                       'PID_Count': PID_count,
                                                                       })
    
#########################################
#                  PID_ES               #
#########################################

@login_required(login_url=settings.LOGIN_URL)
def PID_ES(request):
    return render(request, 'administrator/PID_ES.html')

##Main Module

@login_required(login_url=settings.LOGIN_URL)
def HasilPID_ES(request):
    
    if request.method == 'GET':
        mutation_rate = float(request.GET['mutation_rate'])
        crossover_rate = float(request.GET['crossover_rate'])
        iterasi = int(request.GET['iterasi'])
        pid_search_length = int(request.GET['pid_search_length'])   
             
        motor = DCmotor(J,R,K)
        StPt = 0
    ##############################################################################
    #-----------------------------#Run episode under PID control------------------
    ############################################################################
        Speed = [StPt]         #initialize a vector to keep track of angular velocity
        Voltage = [StPt]        #initalize a vector to keep track of Voltage
        Error = [StPt]
        
        global SetPoint         #initalize a vector to keep track of the error term 
        SetPoint = [StPt]        #initalize a vector to keep track of the setpoint term 
        
        hyper = Hyperactive()

        def PID_evo(opt):
            
            Speed = [StPt]         #initialize a vector to keep track of angular velocity
            Voltage = [StPt]        #initalize a vector to keep track of Voltage
            Error = [StPt]            #initalize a vector to keep track of the error term 
            SetPoint = [StPt]        #initalize a vector to keep track of the setpoint term 
        
            for t in range(episodes):

                #get control move
                Er = (Sp - Speed[t]) #calcualte the error btw setpoint and speed#

                #detirmine the new control position (Voltage)
                V = Voltage[t] + (opt["Kp"]*Er + opt["Ki"]*(Er + Error[t])/2 + \
                opt["Kd"]*(Er-Error[t]))*step_size

                #advacnce motor model by one time step and get new angular velocity
                w = motor.step(Speed[t],V,step_size)

                #append voltage,speed, and error history to their vectors
                Speed = np.append(Speed,w)
                Voltage = np.append(Voltage,V)
                Error = np.append(Error,Er)
                SetPoint = np.append(SetPoint,Sp)
                
            ErrorScore = np.sum(abs(Speed - SetPoint))*step_size
            
            return -ErrorScore
        
        search_space = {
                        "Kp": list(range(0, pid_search_length, 1)),
                        "Ki": list(range(0, pid_search_length, 1)),
                        "Kd": list(range(1, pid_search_length, 1)),
                        }
        
        evo = EvolutionStrategyOptimizer(
                                        mutation_rate = mutation_rate,
                                        crossover_rate = crossover_rate,
                                        rand_rest_p = 0.05,
                                        )
        
        hyper.add_search(PID_evo, search_space, n_iter=iterasi, optimizer=evo,)
        hyper.run()
        
        hasil = hyper.results(PID_evo)
        #hasil.drop(columns = hasil.columns[0], axis=1, inplace=True)
        hasil['score'] = hasil['score'].apply(lambda x: x*-1)
        hasil.to_csv("files/hasil_PID_EVO_{:.4f}.csv".format(-hyper.best_score(PID_evo)))
        
        data_view = []
        
        for x in range(len(hasil['Kp'])):
            temp = []
            temp.append(hasil['Kp'][x])
            temp.append(hasil['Ki'][x])
            temp.append(hasil['Kd'][x])
            temp.append(hasil['eval_time'][x])
            temp.append(hasil['iter_time'][x])
            temp.append(hasil['score'][x])
            
            # print(data['sentiment'][x])
            data_view.append(temp)  
        
        global best_pid_evo
        
        best_pid_evo = hyper.best_para(PID_evo)
        
        for t in range(episodes):

            #get control move
            V,Er = PID_Asli(Error, Speed, Voltage, t, 
                            best_pid_evo.get("Kp"), 
                            best_pid_evo.get("Ki"), 
                            best_pid_evo.get("Kd"), 
                            Sp, step_size)

            #advacnce motor model by one time step and get new angular velocity
            w = motor.step(Speed[t],V,step_size)

            #append voltage,speed, and error history to their vectors
            Speed = np.append(Speed,w)
            Voltage = np.append(Voltage,V)
            Error = np.append(Error,Er)
            SetPoint = np.append(SetPoint,Sp)
            
        SumE_PID_ES = np.sum(abs(Speed - SetPoint))*step_size
        Max_PID_ES = np.max(Voltage)
        PID_count_ES = reversal_count(Voltage, episodes)
        
        #print(SumE_PID)
        #print(Max_PID)
        #print(PID_count)
        
        fig_report = go.Figure()
        fig_report.add_trace(go.Scatter(x=list(range(1, episodes)),
                                        y = SetPoint,
                                        mode='lines',
                                        name='SetPoint'
                                        ))
        
        fig_report.add_trace(go.Scatter(x=list(range(1, episodes)),
                                        y = Speed,
                                        mode='lines',
                                        name='PID ES'
                                        ))
        
        fig_report.update_layout(
                            title="System Response",
                            xaxis_title="Time (.01s)",
                            yaxis_title="Angular Velocity",
                            #legend_title="Method",
                            font=dict(
                                family="Courier New, monospace",
                                size=18,
                                color="RebeccaPurple"
                            )
                        )
        
        fig_PID = plot(fig_report, output_type='div')
    
            
        return render(request, 'administrator/HasilPID_ES.html', context={'plot_PID': fig_PID,
                                                                'Error' : SumE_PID_ES,
                                                                'Max_PID':Max_PID_ES,
                                                                'PID_Count': PID_count_ES,
                                                                'Kp': best_pid_evo.get('Kp'),
                                                                'Ki': best_pid_evo.get('Ki'),
                                                                'Kd': best_pid_evo.get('Kd'),
                                                                'hasil':data_view,
                                                                })
    

#########################################
#                  PID_PSO              #
#########################################

@login_required(login_url=settings.LOGIN_URL)
def PID_PSO(request):
    return render(request, 'administrator/PID_PSO.html')

##Main Module

@login_required(login_url=settings.LOGIN_URL)
def HasilPID_PSO(request):
    
    if request.method == 'GET':
        inertia = float(request.GET['inertia'])
        cognitive_weight = float(request.GET['cognitive_weight'])
        social_weight = float(request.GET['social_weight'])
        temp_weight = float(request.GET['temp_weight'])
        iterasi = int(request.GET['iterasi'])
        pid_search_length = int(request.GET['pid_search_length'])   
             
        motor = DCmotor(J,R,K)
        StPt = 0
    ##############################################################################
    #-----------------------------#Run episode under PID control------------------
    ############################################################################
        Speed = [StPt]         #initialize a vector to keep track of angular velocity
        Voltage = [StPt]        #initalize a vector to keep track of Voltage
        Error = [StPt]            #initalize a vector to keep track of the error term 
        SetPoint = [StPt]        #initalize a vector to keep track of the setpoint term 
        
        hyper = Hyperactive()

        def PID_pso(opt):
            
            Speed = [StPt]         #initialize a vector to keep track of angular velocity
            Voltage = [StPt]        #initalize a vector to keep track of Voltage
            Error = [StPt]            #initalize a vector to keep track of the error term 
            SetPoint = [StPt]        #initalize a vector to keep track of the setpoint term 
        
            for t in range(episodes):

                #get control move
                Er = (Sp - Speed[t]) #calcualte the error btw setpoint and speed#

                #detirmine the new control position (Voltage)
                V = Voltage[t] + (opt["Kp"]*Er + opt["Ki"]*(Er + Error[t])/2 + \
                opt["Kd"]*(Er-Error[t]))*step_size

                #advacnce motor model by one time step and get new angular velocity
                w = motor.step(Speed[t],V,step_size)

                #append voltage,speed, and error history to their vectors
                Speed = np.append(Speed,w)
                Voltage = np.append(Voltage,V)
                Error = np.append(Error,Er)
                SetPoint = np.append(SetPoint,Sp)
                
            ErrorScore = np.sum(abs(Speed - SetPoint))*step_size
            
            return -ErrorScore
        
        search_space = {
                        "Kp": list(range(0, pid_search_length, 1)),
                        "Ki": list(range(0, pid_search_length, 1)),
                        #"Kd": list(np.arange(0.1, 1, 0.1)),
                        "Kd": list(range(1, pid_search_length, 1)),
                        }
        
        PSO = ParticleSwarmOptimizer(
                                    inertia=inertia,
                                    cognitive_weight=cognitive_weight,
                                    social_weight=social_weight,
                                    temp_weight=temp_weight,
                                    rand_rest_p=0.05,
                                    )
        
        hyper.add_search(PID_pso, search_space, n_iter=iterasi, optimizer=PSO,)
        hyper.run()
        
        hasil = hyper.results(PID_pso)
        #hasil.drop(columns = hasil.columns[0], axis=1, inplace=True)
        hasil['score'] = hasil['score'].apply(lambda x: x*-1)
        hasil.to_csv("files/hasil_PID_PSO_{:.4f}.csv".format(-hyper.best_score(PID_pso)))
        
        data_view = []
        
        for x in range(len(hasil['Kp'])):
            temp = []
            temp.append(hasil['Kp'][x])
            temp.append(hasil['Ki'][x])
            temp.append(hasil['Kd'][x])
            temp.append(hasil['eval_time'][x])
            temp.append(hasil['iter_time'][x])
            temp.append(hasil['score'][x])
            
            # print(data['sentiment'][x])
            data_view.append(temp)  
        
        global best_pid_pso
        
        best_pid_pso = hyper.best_para(PID_pso)
        
        for t in range(episodes):

            #get control move
            V,Er = PID_Asli(Error, Speed, Voltage, t, 
                            best_pid_pso.get("Kp"), 
                            best_pid_pso.get("Ki"), 
                            best_pid_pso.get("Kd"), 
                            Sp, step_size)

            #advacnce motor model by one time step and get new angular velocity
            w = motor.step(Speed[t],V,step_size)

            #append voltage,speed, and error history to their vectors
            Speed = np.append(Speed,w)
            Voltage = np.append(Voltage,V)
            Error = np.append(Error,Er)
            SetPoint = np.append(SetPoint,Sp)
            
        SumE_PID_PSO = np.sum(abs(Speed - SetPoint))*step_size
        Max_PID_PSO = np.max(Voltage)
        PID_count_PSO = reversal_count(Voltage, episodes)
        
        #print(SumE_PID)
        #print(Max_PID)
        #print(PID_count)
        
        fig_report = go.Figure()
        fig_report.add_trace(go.Scatter(x=list(range(1, episodes)),
                                        y = SetPoint,
                                        mode='lines',
                                        name='SetPoint'
                                        ))
        
        fig_report.add_trace(go.Scatter(x=list(range(1, episodes)),
                                        y = Speed,
                                        mode='lines',
                                        name='PID PSO'
                                        ))
        
        fig_report.update_layout(
                            title="System Response",
                            xaxis_title="Time (.01s)",
                            yaxis_title="Angular Velocity",
                            #legend_title="Method",
                            font=dict(
                                family="Courier New, monospace",
                                size=18,
                                color="RebeccaPurple"
                            )
                        )
        
        
        fig_PID = plot(fig_report, output_type='div')
    
            
        return render(request, 'administrator/HasilPID_PSO.html', context={'plot_PID': fig_PID,
                                                                'Error' : SumE_PID_PSO,
                                                                'Max_PID':Max_PID_PSO,
                                                                'PID_Count': PID_count_PSO,
                                                                'Kp': best_pid_pso.get('Kp'),
                                                                'Ki': best_pid_pso.get('Ki'),
                                                                'Kd': best_pid_pso.get('Kd'),
                                                                'hasil':data_view,
                                                                })
        
#########################################
#                  PID_EVO_PSO          #
#########################################

@login_required(login_url=settings.LOGIN_URL)
def PID_EVO_PSO(request):
    return render(request, 'administrator/PID_EVO_PSO.html')

##Main Module

@login_required(login_url=settings.LOGIN_URL)
def HasilPID_EVO_PSO(request):
    
    if request.method == 'GET':
        inertia = float(request.GET['inertia'])
        cognitive_weight = float(request.GET['cognitive_weight'])
        social_weight = float(request.GET['social_weight'])
        temp_weight = float(request.GET['temp_weight'])
        iterasi = int(request.GET['iterasi'])
        pid_search_length = int(request.GET['pid_search_length'])   
             
        motor = DCmotor(J,R,K)
        StPt = 0
    ##############################################################################
    #-----------------------------#Run episode under PID control------------------
    ############################################################################
        Speed = [StPt]         #initialize a vector to keep track of angular velocity
        Voltage = [StPt]        #initalize a vector to keep track of Voltage
        Error = [StPt]            #initalize a vector to keep track of the error term 
        SetPoint = [StPt]        #initalize a vector to keep track of the setpoint term 
        
        hyper = Hyperactive()
        def meta_ops(para):
            
            def PID_evo_pso(opt):
                
                Speed = [StPt]         #initialize a vector to keep track of angular velocity
                Voltage = [StPt]        #initalize a vector to keep track of Voltage
                Error = [StPt]            #initalize a vector to keep track of the error term 
                SetPoint = [StPt]        #initalize a vector to keep track of the setpoint term 
            
                for t in range(episodes):

                    #get control move
                    Er = (Sp - Speed[t]) #calcualte the error btw setpoint and speed#

                    #detirmine the new control position (Voltage)
                    V = Voltage[t] + (opt["Kp"]*Er + opt["Ki"]*(Er + Error[t])/2 + \
                    opt["Kd"]*(Er-Error[t]))*step_size

                    #advacnce motor model by one time step and get new angular velocity
                    w = motor.step(Speed[t],V,step_size)

                    #append voltage,speed, and error history to their vectors
                    Speed = np.append(Speed,w)
                    Voltage = np.append(Voltage,V)
                    Error = np.append(Error,Er)
                    SetPoint = np.append(SetPoint,Sp)
                    
                ErrorScore = np.sum(abs(Speed - SetPoint))*step_size
                
                return -ErrorScore
            
            search_space1 = {
                            "Kp": list(range(0, pid_search_length, 1)),
                            "Ki": list(range(0, pid_search_length, 1)),
                            #"Kd": list(np.arange(0.1, 1, 0.1)),
                            "Kd": list(range(1, pid_search_length, 1)),
                            }
            hyper = Hyperactive(verbosity="progress_bar")
            evo = EvolutionStrategyOptimizer(
                                            mutation_rate = para["mutation_rate"],
                                            crossover_rate = para["crossover_rate"],
                                            rand_rest_p = 0.05
                                            )
            
            hyper.add_search(
                            PID_evo_pso,
                            search_space = search_space1,
                            n_iter = iterasi,
                            optimizer = evo,
                            )
            hyper.run()
            
            skor = hyper.best_score(PID_evo_pso)
            
            #print(hyper.results(PID_evo_pso))
            
            global hasil1
            #hasil1 = hyper.results(PID_evo_pso)
            #hasil1['score'] = hasil1['score'].apply(lambda x: x*-1)
            #hasil1.to_csv("files/hasil_PID_EVO_PSO_{:.4f}.csv".format(skor))
            #print(hyper.best_para(PID_EVO_PSO))
            
            global best_pid_evo_pso
            global pid_evo_pso_score
            
            if pid_evo_pso_score < skor :
                best_pid_evo_pso = hyper.best_para(PID_evo_pso)
                pid_evo_pso_score = skor
                hasil1 = hyper.results(PID_evo_pso)
                hasil1['score'] = hasil1['score'].apply(lambda x: x*-1)
                hasil1.to_csv("files/hasil_PID_EVO_PSO_{:.4f}.csv".format(skor))
            
            else:
                pass
            
            return skor
           
        search_space2 = {
            
                        "mutation_rate": list(np.arange(0.1, 1, 0.1)),
                        "crossover_rate": list(np.arange(0.1, 1, 0.1)),

                        }
        
        PSO = ParticleSwarmOptimizer(
                                    inertia=inertia,
                                    cognitive_weight=cognitive_weight,
                                    social_weight=social_weight,
                                    temp_weight=temp_weight,
                                    rand_rest_p=0.05,
                                    )
            
        hyper.add_search(meta_ops, search_space2, n_iter=iterasi, optimizer=PSO,)
        hyper.run()
        
        hasil2 = hyper.results(meta_ops)
        #hasil.drop(columns = hasil.columns[0], axis=1, inplace=True)
        hasil2['score'] = hasil2['score'].apply(lambda x: x*-1)
        hasil2.to_csv("files/hasil_PSO_only_{:.4f}.csv".format(-hyper.best_score(meta_ops)))
        
        global best_es_var
        best_es_var = hyper.best_para(meta_ops) 
        
        global data_view_evo_pso
        data_view_evo_pso = None
        data_view_evo_pso = []
        
        for x in range(len(hasil1['Kp'])):
            temp1 = None
            temp1 = []
            temp1.append(hasil1['Kp'][x])
            temp1.append(hasil1['Ki'][x])
            temp1.append(hasil1['Kd'][x])
            temp1.append(hasil1['eval_time'][x])
            temp1.append(hasil1['iter_time'][x])
            temp1.append(hasil1['score'][x])
            
            # print(data['sentiment'][x])
            data_view_evo_pso.append(temp1)
        
        #print(data_view_evo_pso)
        
        data_view_pso_value = None
        data_view_pso_value = []
        
        for y in range(len(hasil2['mutation_rate'])):
            temp2 = None
            temp2 = []
            temp2.append(hasil2['mutation_rate'][y])
            temp2.append(hasil2['crossover_rate'][y])
            temp2.append(hasil2['eval_time'][y])
            temp2.append(hasil2['iter_time'][y])
            temp2.append(hasil2['score'][y])
            
            # print(data['sentiment'][x])
            data_view_pso_value.append(temp2)
    
    for t in range(episodes):

        #get control move
        V,Er = PID_Asli(Error, Speed, Voltage, t, 
                        best_pid_evo_pso.get("Kp"), 
                        best_pid_evo_pso.get("Ki"), 
                        best_pid_evo_pso.get("Kd"), 
                        Sp, step_size)

        #advacnce motor model by one time step and get new angular velocity
        w = motor.step(Speed[t],V,step_size)

        #append voltage,speed, and error history to their vectors
        Speed = np.append(Speed,w)
        Voltage = np.append(Voltage,V)
        Error = np.append(Error,Er)
        SetPoint = np.append(SetPoint,Sp)
        
    SumE_PID_ES_PSO = np.sum(abs(Speed - SetPoint))*step_size
    Max_PID_ES_PSO = np.max(Voltage)
    PID_count_ES_PSO = reversal_count(Voltage, episodes)
    
    #print(SumE_PID)
    #print(Max_PID)
    #print(PID_count)
    
    fig_report = go.Figure()
    fig_report.add_trace(go.Scatter(x=list(range(1, episodes)),
                                    y = SetPoint,
                                    mode='lines',
                                    name='SetPoint'
                                    ))
    
    fig_report.add_trace(go.Scatter(x=list(range(1, episodes)),
                                    y = Speed,
                                    mode='lines',
                                    name='PID EVO+PSO'
                                    ))
    fig_report.update_layout(
                            title="System Response",
                            xaxis_title="Time (.01s)",
                            yaxis_title="Angular Velocity",
                            #legend_title="Method",
                            font=dict(
                                family="Courier New, monospace",
                                size=18,
                                color="RebeccaPurple"
                            )
                        )
    
    fig_PID = plot(fig_report, output_type='div')

        
    return render(request, 'administrator/HasilPID_EVO_PSO.html', context={
                                                            'plot_PID': fig_PID,
                                                            'Error' : SumE_PID_ES_PSO,
                                                            'Max_PID':Max_PID_ES_PSO,
                                                            'PID_Count': PID_count_ES_PSO,
                                                            'Kp': best_pid_evo_pso.get('Kp'),
                                                            'Ki': best_pid_evo_pso.get('Ki'),
                                                            'Kd': best_pid_evo_pso.get('Kd'),
                                                            'hasil1':data_view_evo_pso,
                                                            'hasil2':data_view_pso_value,
                                                            })


@login_required(login_url=settings.LOGIN_URL)
def Rangkuman(request):
    
    print("EVO+PSO = {}".format(best_pid_evo_pso))
    print("EVO = {}".format(best_pid_evo))
    print("PSO = {}".format(best_pid_pso))
    
    if best_pid_evo_pso == None and best_pid_evo == None and best_pid_pso == None and pid_val == None:
        
        motor = DCmotor(J,R,K)
        StPt = 0
    ##############################################################################
    #-----------------------------#Run episode under PID control------------------
    ############################################################################

        Speed = [StPt]         #initialize a vector to keep track of angular velocity
        Voltage = [StPt]        #initalize a vector to keep track of Voltage
        Error = [StPt]            #initalize a vector to keep track of the error term 
        SetPoint = [StPt]        #initalize a vector to keep track of the setpoint term 
        
        Kp = 1
        Ki = 0
        Kd = 10
        for t in range(episodes):

            #get control move
            V,Er = PID_Asli(Error, Speed, Voltage, t, Kp, Ki, Kd, Sp, step_size)

            #advacnce motor model by one time step and get new angular velocity
            w = motor.step(Speed[t],V,step_size)

            #append voltage,speed, and error history to their vectors
            Speed = np.append(Speed,w)
            Voltage = np.append(Voltage,V)
            Error = np.append(Error,Er)
            SetPoint = np.append(SetPoint,Sp)
        
        fig_report = go.Figure()
        fig_report.add_trace(go.Scatter(x=list(range(1, episodes)),
                                        y = SetPoint,
                                        mode='lines',
                                        name='SetPoint'
                                        ))
        
        fig_report.update_layout(
                            title="System Response",
                            xaxis_title="Time (.01s)",
                            yaxis_title="Angular Velocity",
                            #legend_title="Method",
                            font=dict(
                                family="Courier New, monospace",
                                size=18,
                                color="RebeccaPurple"
                            )
                        )
        
        
        fig_Report = plot(fig_report, output_type='div')

        Error_val = "Null"
        Amp_val = "Null"
        Rev_val = "Null"
        
        render_view = False
        metode1 = "Null"
        messages.error(request,'Data belum di lakukan training Silahkan training terlebih dahulu!')
        
        return render(request, 'administrator/Rangkuman.html', context={ 
                                                                    'render_view':render_view,
                                                                    
                                                                    'figReport':fig_Report,
                                                                    'Metode1':metode1,
                                                                    'Error':Error_val,
                                                                    'Max_PID':Amp_val,
                                                                    'PID_Count':Rev_val,
                                                                    
                                                                    #'Error_evo':Error_val,
                                                                    #'Max_PID_evo':Amp_val,
                                                                    #'PID_Count_evo':Rev_val,
                                                                    
                                                                    #'Error_pso':Error_val,
                                                                    #'Max_PID_pso':Amp_val,
                                                                    #'PID_Count_pso':Rev_val,
                                                                    
                                                                    #'Error_evo_pso':Error_val,
                                                                    #'Max_PID_evo_pso':Amp_val,
                                                                    #'PID_Count_evo_pso':Rev_val,
            

                                                                    })
        
    else:
        motor = DCmotor(J, R, K)
        StPt = 0
        
        Speed = [StPt]
        Voltage= [StPt]
        Error = [StPt]
        SetPoint = [StPt]
        
        Speed_EVO = [StPt]
        Voltage_EVO= [StPt]
        Error_EVO = [StPt]
        SetPoint_EVO = [StPt]
        
        Speed_PSO = [StPt]
        Voltage_PSO = [StPt]
        Error_PSO = [StPt]
        SetPoint_PSO = [StPt]
        
        Speed_EVO_PSO = [StPt]
        Voltage_EVO_PSO = [StPt]
        Error_EVO_PSO = [StPt]
        SetPoint_EVO_PSO = [StPt]
        
        
        for t in range(episodes):

            #get control move
            V,Er = PID_Asli(Error, Speed, Voltage, t, 
                            pid_val.get("Kp"), 
                            pid_val.get("Ki"), 
                            pid_val.get("Kd"), 
                            Sp, step_size)

            #advacnce motor model by one time step and get new angular velocity
            w = motor.step(Speed[t],V,step_size)

            #append voltage,speed, and error history to their vectors
            Speed = np.append(Speed,w)
            Voltage = np.append(Voltage,V)
            Error = np.append(Error,Er)
            SetPoint = np.append(SetPoint,Sp)
            
            V_evo,Er_evo = PID_Asli(Error_EVO, Speed_EVO, Voltage_EVO, t, 
                            best_pid_evo.get("Kp"), 
                            best_pid_evo.get("Ki"), 
                            best_pid_evo.get("Kd"),
                            Sp, step_size)

            #advacnce motor model by one time step and get new angular velocity
            w_evo = motor.step(Speed_EVO[t],V_evo,step_size)

            #append voltage,speed, and error history to their vectors
            Speed_EVO = np.append(Speed_EVO,w_evo)
            Voltage_EVO = np.append(Voltage_EVO,V_evo)
            Error_EVO = np.append(Error_EVO,Er_evo)
            SetPoint_EVO = np.append(SetPoint_EVO,Sp)
            
            V_pso,Er_pso = PID_Asli(Error_PSO, Speed_PSO, Voltage_PSO, t, 
                            best_pid_pso.get("Kp"), 
                            best_pid_pso.get("Ki"), 
                            best_pid_pso.get("Kd"),
                            Sp, step_size)

            #advacnce motor model by one time step and get new angular velocity
            w_pso = motor.step(Speed_PSO[t],V_pso,step_size)

            #append voltage,speed, and error history to their vectors
            Speed_PSO = np.append(Speed_PSO,w_pso)
            Voltage_PSO = np.append(Voltage_PSO,V_pso)
            Error_PSO = np.append(Error_PSO,Er_pso)
            SetPoint_PSO = np.append(SetPoint_PSO,Sp)
            
            V_evo_pso, Er_evo_pso = PID_Asli(Error_EVO_PSO, Speed_EVO_PSO, Voltage_EVO_PSO, t, 
                            best_pid_evo_pso.get("Kp"), 
                            best_pid_evo_pso.get("Ki"), 
                            best_pid_evo_pso.get("Kd"),
                            Sp, step_size)

            #advacnce motor model by one time step and get new angular velocity
            w_evo_pso = motor.step(Speed_EVO_PSO[t],V_evo_pso,step_size)

            #append voltage,speed, and error history to their vectors
            Speed_EVO_PSO = np.append(Speed_EVO_PSO,w_evo_pso)
            Voltage_EVO_PSO = np.append(Voltage_EVO_PSO,V_evo_pso)
            Error_EVO_PSO = np.append(Error_EVO_PSO,Er_evo_pso)
            SetPoint_EVO_PSO = np.append(SetPoint_EVO_PSO,Sp)
            
            
            
        SumE_PID = np.sum(abs(Speed - SetPoint))*step_size
        Max_PID = np.max(Voltage)
        PID_count = reversal_count(Voltage, episodes)
        
        SumE_PID_EVO = np.sum(abs(Speed_EVO - SetPoint))*step_size
        Max_PID_EVO = np.max(Voltage_EVO)
        PID_count_EVO = reversal_count(Voltage_EVO, episodes)
        
        SumE_PID_PSO = np.sum(abs(Speed_PSO - SetPoint))*step_size
        Max_PID_PSO = np.max(Voltage_PSO)
        PID_count_PSO = reversal_count(Voltage_PSO, episodes)
        
        SumE_PID_EVO_PSO = np.sum(abs(Speed_EVO_PSO - SetPoint))*step_size
        Max_PID_EVO_PSO = np.max(Voltage_EVO_PSO)
        PID_count_EVO_PSO = reversal_count(Voltage_EVO_PSO, episodes)
        
        fig_report = go.Figure()
        fig_report.add_trace(go.Scatter(x=list(range(1, episodes)),
                                        y = SetPoint,
                                        mode='lines',
                                        name='SetPoint'
                                        ))
        
        fig_report.add_trace(go.Scatter(x=list(range(1, episodes)),
                                        y = Speed,
                                        mode='lines',
                                        name='PID'
                                        ))
        
        fig_report.add_trace(go.Scatter(x=list(range(1, episodes)),
                                        y = Speed_EVO,
                                        mode='lines',
                                        name='PID + EVO'
                                        ))
        
        fig_report.add_trace(go.Scatter(x=list(range(1, episodes)),
                                        y = Speed_PSO,
                                        mode='lines',
                                        name='PID + PSO'
                                        ))
        
        fig_report.add_trace(go.Scatter(x=list(range(1, episodes)),
                                        y = Speed_EVO_PSO,
                                        mode='lines',
                                        name='PID + EVO PSO'
                                        ))
        fig_report.update_layout(
                            title="System Response",
                            xaxis_title="Time (.01s)",
                            yaxis_title="Angular Velocity",
                            #legend_title="Method",
                            font=dict(
                                family="Courier New, monospace",
                                size=18,
                                color="RebeccaPurple"
                            )
                        )     
        
        
        fig_Report = plot(fig_report, output_type='div')
        
        render_view = True
        metode1 = "PID"
        metode2 = "PID EVO"
        metode3 = "PID PSO"
        metode4 = "PID EVO+PSO"
        
        t = list(range(1, episodes))
        info_pid = step_info(Speed, t)
        info_evo = step_info(Speed_EVO, t)
        info_pso = step_info(Speed_PSO, t)
        info_evo_pso = step_info(Speed_EVO_PSO, t)
                        
        #return redirect('administrator/overall')
        return render(request, 'administrator/Rangkuman.html', context={
                                                                        'render_view':render_view,

                                                                        'figReport':fig_Report,
                                                                        
                                                                        'Metode1':metode1,
                                                                        'Error':SumE_PID,
                                                                        'Max_PID':Max_PID,
                                                                        'PID_Count':PID_count,
                                                                        'info_pid':info_pid,
                                                                        
                                                                        'Metode2':metode2,
                                                                        'Error_evo':SumE_PID_EVO,
                                                                        'Max_PID_evo':Max_PID_EVO,
                                                                        'PID_Count_evo':PID_count_EVO,
                                                                        'info_evo':info_evo,
                                                                        
                                                                        'Metode3':metode3,
                                                                        'Error_pso':SumE_PID_PSO,
                                                                        'Max_PID_pso':Max_PID_PSO,
                                                                        'PID_Count_pso':PID_count_PSO,
                                                                        'info_pso':info_pso,
                                                                        
                                                                        'Metode4':metode4,
                                                                        'Error_evo_pso':SumE_PID_EVO_PSO,
                                                                        'Max_PID_evo_pso':Max_PID_EVO_PSO,
                                                                        'PID_Count_evo_pso':PID_count_EVO_PSO,
                                                                        'info_evo_pso':info_evo_pso,
                                                                        
                                                                        
                                                                        
                                                                        
            
                                                                        })
