import socket
import time
import json

LOGSTASH_IP = "192.168.0.76"
LOGSTASH_PORT = 5000

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

def get_arp_table():
    arp_entries = []
    with open("/proc/net/arp") as f:
        next(f)
        for line in f:
            parts = line.split()
            if len(parts) >= 6:
                arp_entries.append({
                    "ip": parts[0],
                    "mac": parts[3],
                    "interface": parts[5]
                })
    return arp_entries

while True:
    payload = {
        "arp_host": "pi4",
        "timestamp": time.time(),
        "arp_table": get_arp_table()
    }

    data = json.dumps(payload).encode()
    sock.sendto(data, (LOGSTASH_IP, LOGSTASH_PORT))
    print("[+] Sent ARP data over UDP")
    time.sleep(5)
