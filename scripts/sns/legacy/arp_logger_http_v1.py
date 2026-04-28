import requests
import time
import json
import socket

LOGSTASH_URL = "http://192.168.0.76:5000"  # <-- change this

def get_hostname():
    return socket.gethostname()

def get_arp_table():
    arp_entries = []

    with open("/proc/net/arp") as f:
        next(f)  # skip header
        for line in f:
            parts = line.split()

            arp_entries.append({
                "ip": parts[0],
                "mac": parts[3],
                "interface": parts[5]
            })

    return arp_entries


def send_arp():
    payload = {
        "host": get_hostname(),
        "timestamp": time.time(),
        "arp_table": get_arp_table()
    }

    try:
        r = requests.post(LOGSTASH_URL, json=payload, timeout=2)
        print(f"[+] Sent ARP data: {r.status_code}")
    except Exception as e:
        print(f"[!] Error sending data: {e}")


if __name__ == "__main__":
    while True:
        send_arp()
        time.sleep(5)
