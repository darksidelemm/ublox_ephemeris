import datetime

iono_data = {'utcA0': -2.7939677238464355e-09, 'utcA1': -6.217248937900877e-15, 'utcWNT': 1980, 'utcLSF': 18, 'utcLS': 18, 'utcDN': 7, 'utcSpare': 0, 'klobB2': -131072.0, 'klobB3': 917504.0, 'klobB0': 96256.0, 'klobB1': -147456.0, 'health': 4294967287, 'utcWNF': 1929, 'klobA3': 1.1920928955078125e-07, 'klobA2': -5.960464477539063e-08, 'utcTOW': 61440, 'klobA0': 9.313225746154785e-09, 'klobA1': -1.4901161193847656e-08, 'flags': 7}

def generate_rinex_navigation_header(program="VK5QI PYTHON", 
                                    run_by="VK5QI", 
                                    date = datetime.datetime.utcnow(), 
                                    comment="IGS BROADCAST EPHEMERIS FILE",
                                    iono_data = iono_data):
    output =  "0         1         2         3         4         5         6         7         8\n"
    output += "012345678901234567890123456789012345678901234567890123456789012345678901234567890\n"
    output += "%9.2f%11s%-20s%20s%-20s\n" % (2,'','NAVIGATION DATA','',"RINEX VERSION / TYPE")
    output += "%-20s%-20s%-20s%-20s\n" % (program, run_by, date.strftime("%d-%b-%y %H:%M"),"PGM / RUN BY / DATE")
    output += "%-60s%-20s\n" % (comment,"COMMENT")
    output += "%2s%12.4E%12.4E%12.4E%12.4E%10s%-20s\n" % ('',iono_data['klobA0'],iono_data['klobA1'],iono_data['klobA2'],iono_data['klobA3'],'','ION ALPHA')
    output += "%2s%12.4E%12.4E%12.4E%12.4E%10s%-20s\n" % ('',iono_data['klobB0'],iono_data['klobB1'],iono_data['klobB2'],iono_data['klobB3'],'','ION BETA')
    output += "%3s%19.12E%19.12E%9d%9d %-20s\n" % ('',iono_data['utcA0'],iono_data['utcA1'],iono_data['utcTOW'],iono_data['utcWNT'],'DELTA-UTC: A0,A1,T,W')
    output += "%6d%54s%-20s\n" % (iono_data['utcLS'],'','LEAP SECONDS')
    output += "%60s%-20s" % ('','END OF HEADER')

    return output

generate_rinex_navigation_header()