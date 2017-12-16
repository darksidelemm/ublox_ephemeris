#!/usr/bin/env python
#
#   uBlox Broadcast Ephemeris Extractor
#
#   Mark Jessop <vk5qi@rfhead.net>
#
#   References:
#       RINEX Format Specification - ftp://igs.org/pub/data/format/rinex210.txt
#       Example RINEX Navigation Message File - http://rfhead.net/sondes/ephemeris.dat
#
#

from ublox import *
from ephemeris import *
import sys, time, datetime, math, argparse


# Dictionary to hold SV Ephemerides.
sv_ephem = {}
iono_data = None


def generate_rinex_navigation_header(program="VK5QI PYTHON", 
                                    run_by="VK5QI", 
                                    date = datetime.datetime.utcnow(), 
                                    comment="EPHEMERIDES FROM UBLOX GPS",
                                    iono_data = iono_data):
    '''
    Generate a RINEX Navigation Data header block.
    '''
    #output =  "0         1         2         3         4         5         6         7         8\n"
    #output += "012345678901234567890123456789012345678901234567890123456789012345678901234567890\n"
    output = "%9.2f%11s%-20s%20s%-20s\n" % (2, '', 'NAVIGATION DATA', '', "RINEX VERSION / TYPE")
    output += "%-20s%-20s%-20s%-20s\n" % (program, run_by, date.strftime("%d-%b-%y %H:%M"), "PGM / RUN BY / DATE")
    output += "%-60s%-20s\n" % (comment, "COMMENT")
    output += "%2s%12.4E%12.4E%12.4E%12.4E%10s%-20s\n" % (
        '', iono_data['klobA0'], iono_data['klobA1'], iono_data['klobA2'], iono_data['klobA3'], '', 'ION ALPHA')
    output += "%2s%12.4E%12.4E%12.4E%12.4E%10s%-20s\n" % (
        '', iono_data['klobB0'], iono_data['klobB1'], iono_data['klobB2'], iono_data['klobB3'], '', 'ION BETA')
    output += "%3s%19.12E%19.12E%9d%9d %-20s\n" % (
        '', iono_data['utcA0'], iono_data['utcA1'], iono_data['utcTOW'], iono_data['utcWNT'], 'DELTA-UTC: A0,A1,T,W')
    output += "%6d%54s%-20s\n" % (iono_data['utcLS'], '', 'LEAP SECONDS')
    output += "%60s%-20s\n" % ('', 'END OF HEADER')

    return output

def generate_ephemeris_block(ephem):
    '''
    Generate a RINEX Navigation Message ephemeris data block, using data
    from a pyUblox EphemerisData object
    Reference: ftp://igs.org/pub/data/format/rinex210.txt Table A4
    '''
    # TODO: Calculate Ephemeris Epoch Time from toc data.
    output = "%2d %2d %2d %2d %2d %2d%5.1f%19.12E%19.12E%19.12E\n" % (ephem.svid, 0,0,0,0,0,0, ephem.af0, ephem.af1, ephem.af2)  # TODO - Decipher SV Epoch time.
    output += "   %19.12E%19.12E%19.12E%19.12E\n" % (ephem.iode, ephem.crs, ephem.deltaN, ephem.M0)
    output += "   %19.12E%19.12E%19.12E%19.12E\n" % (ephem.cuc, ephem.ecc, ephem.cus, math.sqrt(ephem.A))
    output += "   %19.12E%19.12E%19.12E%19.12E\n" % (ephem.toe, ephem.cic, ephem.omega0, ephem.cis)
    output += "   %19.12E%19.12E%19.12E%19.12E\n" % (ephem.i0, ephem.crc, ephem.omega, ephem.omega_dot)
    output += "   %19.12E%19.12E%19.12E%19.12E\n" % (ephem.idot, ephem.code_on_l2, ephem.week_no, ephem.l2_p_flag)
    # TODO: Check sv_health field is OK.
    output += "   %19.12E%19.12E%19.12E%19.12E\n" % (ephem.sv_ura, ephem.sv_health, ephem.Tgd, ephem.iodc)
    # TODO: Calculate message transmission time from hand-over word. (First element below)
    output += "   %19.12E%19.12E%19.12E%19.12E\n" % (0, 0, 0, 0)

    return output 

def handle_messages(msg):
        global sv_ephem, iono_data
        msg.unpack()
        if msg.name() == "AID_HUI":
            print("Received Ionospheric Data.")
            iono_data = msg._fields

        elif msg.name() == "AID_EPH":
            eph_data = EphemerisData(msg)
            if eph_data.valid:
                sv_ephem[eph_data.svid] = eph_data
                print("Received valid Ephemeris Data for SVID: %d" % eph_data.svid)
        else:
            pass

if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("-p" ,"--port", default="/dev/ttyUSB0", help="GPS Serial port (i.e. /dev/ttyUSB0 or COM5)")
    parser.add_argument("-b", "--baudrate", type=int, default=115200, help="GPS Baud Rate")
    parser.add_argument("-o", "--output", default="ephemeris.dat", help="Output file (Default: ephemeris.dat)")
    parser.add_argument("-t", "--timeout", type=int, default=30, help="Wait for sats timeout (seconds)")
    parser.add_argument("-s", "--minsats", type=int, default=6, help="Minimum number ov SVs required.")
    args = parser.parse_args()

    MIN_SATS = args.minsats

    # Open connection to GPS
    ubx = UBloxGPS(port=args.port, baudrate=args.baudrate, callback=handle_messages, update_rate_ms=1000)

    print("Connected, waiting for enough sats.")
    timeout = args.timeout
    while timeout > 0:
        current_state = ubx.read_state()
        print("Currently tracking %d SVs." % current_state['numSV'])
        if current_state['numSV'] >= MIN_SATS:
            break
        time.sleep(1)
        timeout -= 1

    if timeout == 0:
        print("Timed out waiting for SVs.")
        sys.exit(1)
    else:
        print("Tracking enough SVs... polling for Almanac/Ephemerides.")

    # Poll for Ionosphere Data
    ubx.gps.send_message(CLASS_AID, MSG_AID_HUI,'')
    time.sleep(1)
    # Poll for Ephemeris Data.
    ubx.gps.send_message(CLASS_AID, MSG_AID_EPH,'')
    # Wait a few seconds to be sure we have collected all the data.
    time.sleep(5)
    # Close the connection to the GPS.
    ubx.close()

    output_data = generate_rinex_navigation_header(iono_data=iono_data)
    for _sv in sv_ephem.keys():
        output_data += generate_ephemeris_block(sv_ephem[_sv])

    print("Writing Ephemeris Data to File: %s" % args.output)

    out_f = open(args.output, 'w')
    out_f.write(output_data)
    out_f.close()

    print(output_data)


