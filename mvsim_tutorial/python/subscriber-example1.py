#!/usr/bin/env python

import pymvsim_comms

if __name__ == "__main__":
    client = pymvsim_comms.mvsim.Client()
    # client.setName("listener1")
    print("Connecting to server...")
    client.connect()
    print("Connected successfully.")
