import requests
import time

LOGSTASH_URL = "http://192.168.0.76:5000"

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

def send_arp():
    payload = {
        "arp_host": "pi4",
        "timestamp": time.time(),
        "arp_table": get_arp_table()
    }

    try:
        r = requests.post(LOGSTASH_URL, json=payload, timeout=5)
        print("[+] Sent ARP data", r.status_code, r.text)
    except Exception as e:
        print("[!] Error:", e)

while True:
    send_arp()
    time.sleep(5)
