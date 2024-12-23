import subprocess

result = subprocess.Popen(["roslaunch journal_rendezvous test.launch"], shell=True)
print("CIAO")