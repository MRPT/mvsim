#!/usr/bin/env python

import pymvsim_comms
import time

if __name__ == "__main__":
    client = pymvsim_comms.mvsim.Client()
    client.setName("tutorial1")
    print("Connecting to server...")
    client.connect()
    print("Connected successfully.")
    time.sleep(2.0)
