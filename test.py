import time
import subprocess

def sync_time_with_ntp():
    try:
        subprocess.run(['sudo', 'ntpdate', '-s', 'time.google.com'], check=True, capture_output=True)
        print("Time synchronized with NTP server.")
    except subprocess.CalledProcessError as e:
        print(f"Error synchronizing time: {e}")

def get_synced_time_ns():
     return time.time_ns()

#On both computers
sync_time_with_ntp()
timestamp = get_synced_time_ns()
print(f"Current time in nanoseconds: {timestamp}")