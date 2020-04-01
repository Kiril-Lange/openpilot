#!/usr/bin/env python3
"""
This tool can be used to quickly changes the values in a JSON file used for tuning
Keys like in vim:
 - h: decrease by 0.01
 - l: increase by 0.01
 - k: move pointer up
 - j: move pointer down
"""

import os
import tty
import sys
import json
import termios
from collections import OrderedDict

FILENAME = '/data/tuning.json'

def read_tuning():
  while True:
    try:
      return json.loads(open(FILENAME).read())
    except:
      pass

def main():
  os.system('clear')
  dat = json.loads(open(FILENAME, 'r').read())
  dat = OrderedDict(sorted(dat.items(), key=lambda i: i[0]))

  cur = 0
  while True:
    sys.stdout.write("\x1Bc")

    for i, (k, v) in enumerate(dat.items()):
      if(k != "max_range"):
        prefix = "> " if cur == i else "  "
        print((prefix + k).ljust(20) + "%.2f" % v)

    key = sys.stdin.read(1)[0]

    write = False
    if key == "k":
      cur = max(0, cur - 1)
    elif key == "j":
      cur = min(len(dat.keys()) - 1, cur + 1)
    elif key == "l":
      dat[list(dat.keys())[cur]] += 0.01
      write = True
    elif key == "h":
      dat[list(dat.keys())[cur]] -= 0.01
      write = True
    elif key == "q":
      break

    if write:
      open(FILENAME, 'w').write(json.dumps(dat))


if __name__ == "__main__":
  orig_settings = termios.tcgetattr(sys.stdin)
  tty.setcbreak(sys.stdin)

  try:
    main()
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, orig_settings)
  except:
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, orig_settings)
    raise
