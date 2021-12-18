from django.urls import path, include
from . import views

app_name = 'administrator'

urlpatterns = [
    path('', views.index, name='index'),
    path('ParamPID', views.ParamPID, name='ParamPID'),
    
    path('PID/', views.PID, name='PID'),
    path('HasilPID/', views.HasilPID, name='HasilPID'),
    
    path('PID_ES/', views.PID_ES, name='PID_ES'),
    path('HasilPID_ES/', views.HasilPID_ES, name='HasilPID_ES'),
    
    path('PID_PSO/', views.PID_PSO, name='PID_PSO'),
    path('HasilPID_PSO/', views.HasilPID_PSO, name='HasilPID_PSO'),
    
    path('PID_EVO_PSO/', views.PID_EVO_PSO, name='PID_EVO_PSO'),
    path('HasilPID_EVO_PSO/', views.HasilPID_EVO_PSO, name='HasilPID_EVO_PSO'),
    
    path('Rangkuman', views.Rangkuman, name='Rangkuman'),
   
    path('tentang/', views.tentang, name='tentang'),
   
    
    
   



]
