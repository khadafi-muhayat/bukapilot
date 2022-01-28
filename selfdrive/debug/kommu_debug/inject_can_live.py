#!/usr/bin/env python3
from panda import Panda
import time

# list format [addr, msg, bus, freq]
tx = [0x1d0, b'\xff\x01\x10@\x00\x00\x00)', 0, 10 ]
spam_time_s = 1 # don't inject more than 5 seconds to prevent dangerous lockouts

if __name__ == "__main__":
  p = Panda()
  p.set_safety_mode(Panda.SAFETY_ALLOUTPUT)

  start_time = time.time()

  # get all the frequencies and group the unique freqs
  freqs = 10

  # group tx messages into sending order
  tx_msgs = []
 # for i, f in enumerate(freqs):
  #  for m in tx:
   #   tx_msgs[0].append(m)

  # get the sending dt
  send_time_s = 1/10
#  send_time_subtracted = []
 # for i in range(len(send_time_s)):
  #  send_time_subtracted.append(send_time_s[i] - sum(send_time_s[:i]))

  print("Spamming all buses...")
  while True:

    if time.time() - start_time > spam_time_s:
      break

    p.can_send(tx[0], tx[1], tx[2])
    time.sleep(0.001)


  p.set_safety_mode(Panda.SAFETY_SILENT)
