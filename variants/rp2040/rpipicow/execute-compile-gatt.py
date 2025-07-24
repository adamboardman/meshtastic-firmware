#!/usr/bin/env python3
import sys
import os
import subprocess
import json
import re

Import("env")

if os.path.getmtime("src/platform/rp2xx0/meshtastic-profile.gatt") > os.path.getmtime("src/platform/rp2xx0/include/meshtastic-profile.h"):
    env.Execute("~/.platformio/packages/framework-arduinopico/pico-sdk/lib/btstack/tool/compile_gatt.py "
            " src/platform/rp2xx0/meshtastic-profile.gatt"
            " src/platform/rp2xx0/include/meshtastic-profile.h")
