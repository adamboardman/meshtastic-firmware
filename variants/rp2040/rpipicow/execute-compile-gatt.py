#!/usr/bin/env python3
import sys
from os.path import join
import subprocess
import json
import re

Import("env")

env.Execute("~/.platformio/packages/framework-arduinopico/pico-sdk/lib/btstack/tool/compile_gatt.py "
            " src/platform/rp2xx0/meshtastic-profile.gatt"
            " src/platform/rp2xx0/include/meshtastic-profile.h")
