import subprocess

ret=subprocess.check_output(['../../x64/Release/NHKrobocon.exe'])
print(ret)