import paramiko
import subprocess

def listen():
    # Create an SSH client
    ssh = paramiko.SSHClient()

    # Automatically add the Raspberry Pi's host key
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())

    # Connect to the Raspberry Pi
    ssh.connect("192.168.2.2", username="raspberrypi4", password="raspberrypi4")

    # Execute commands over SSH
    command="tmux new-session -d -s mysession 'python3 mission.py'"
    stdin, stdout, stderr = ssh.exec_command(command)
    # Print the output
    print(stderr.read().decode())
    print(stdout.read().decode())



    # Close the SSH connection
    ssh.close()


def download():
    # Command to execute in CMD
    command = "start cmd /k scp -r raspberrypi4@192.168.2.2:/home/raspberrypi4/mission C:\\Users\\PC\\Desktop"

    # Execute the command in CMD
    subprocess.call(command, shell=True)

