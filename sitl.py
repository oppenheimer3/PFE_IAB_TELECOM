import os
import subprocess
def run_simulation():
    file1="C:\\Users\\PC\\documents\\Mission Planner\\sitl"
    file2="C:\\Users\\PC\\Documents\\Mission Planner\\sitl\\default_params\\copter.parm"
    os.chdir(file1)
    subprocess.Popen("ArduCopter.exe --model copter --defaults \""+file2+"\" --home 36.50009746958337,2.8782173307732744,213,0" , creationflags=subprocess.CREATE_NEW_CONSOLE)
if __name__=='__main__':
    run_simulation()