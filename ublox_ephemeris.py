#!/usr/bin/env python
#
#   uBlox Broadcast Ephemeris Extractor
#
#   Mark Jessop <vk5qi@rfhead.net>
#

from ublox import *
from ephemeris import *
import sys, time


MIN_SATS = 5

# Dictionary to hold SV Ephemerides.
sv_ephem = {}
iono_data = None


if __name__ == "__main__":
    """ Basic test script for the above UBloxGPS class. 
    Sets up GPS and prints out basic position information.
    """
    import sys

    def handle_messages(msg):
        msg.unpack()
        if msg.name() == "AID_HUI":
            print("Got Ionospheric Data.")
            print(msg._fields)
            print(msg._recs)

        elif msg.name() == "AID_EPH":
            eph_data = EphemerisData(msg)
            if eph_data.valid:
                sv_ephem[eph_data.svid] = eph_data
                print("Received valid Ephemeris Data for SVID: %d" % eph_data.svid)
        else:
            #print(msg.name())
            pass


    # Open connection to GPS
    ubx = UBloxGPS(port=sys.argv[1], callback=handle_messages, update_rate_ms=1000)

    print("Connected, waiting for enough sats.")
    got_lock = False
    while not got_lock:
        current_state = ubx.read_state()
        print("Currently tracking %d SVs." % current_state['numSV'])
        if current_state['numSV'] >= MIN_SATS:
            got_lock = True
        time.sleep(1)

    # Poll for Ionosphere Data
    ubx.gps.send_message(CLASS_AID, MSG_AID_HUI,'')
    time.sleep(1)
    # Poll for Ephemeris Data.
    ubx.gps.send_message(CLASS_AID, MSG_AID_EPH,'')

    time.sleep(5)
    ubx.close()

