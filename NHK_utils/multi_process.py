import sys
import subprocess

def get_lines(cmd):
    '''
    :param cmd: str 実行するコマンド
    :rtype: generator
    :return: 標準出力（行ごと）
    '''
    proc=subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)

    while True:
        line=proc.stdout.readline().decode('utf-8')
        if line:
            yield line
        if not line and proc.poll() is not None: #proc.poll() check if child process has terminated
            break