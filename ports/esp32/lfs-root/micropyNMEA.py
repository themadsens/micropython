
from micropyGPS import MicropyGPS

class MicropyNMEA(MicropyGPS):
    def __init__(self, local_offset=0, location_formatting='ddm'):
        super().__init__(local_offset, location_formatting)
        self.depth_below_surface = None
        self.depth_below_transducer = None
        self.relative_wind_angle = None
        self.relative_wind_speed = None
        self.true_wind_angle = (None, None)
        self.true_wind_speed = None
        self.barometric_pressure = None
        self.air_temperature = None
        self.water_temperature = None
        self.relative_humidity = None
        self.dew_point = None
        self.time_offset_minutes = None


    def gpvwr(self):
        # $YDVWR,140.9,R,12.4,N,6.4,M,22.9,K*54
        # $YDVWR,103.4,R,11.8,N,6.1,M,21.8,K*56
        try:
            direction = float(self.gps_segments[1])
            direction *= -1 if self.gps_segments[2] == "L" else 1
            speed = float(self.gps_segments[7]) / 3.6 # KpH is best accuracy
        except ValueError: pass
        else:
            self.relative_wind_speed = speed
            self.relative_wind_angle = direction
            return True

        return

    def gpmda(self):
        # $YDMDA,,I,,B,,C,17.9,C,,,,C,279.5,T,275.5,M,14.0,N,7.2,M*2A
        parsed = None
        try:
            value = float(self.gps_segments[3])
        except ValueError: pass
        else:
            self.barometric_pressure = value
            parsed = True

        try:
            value = float(self.gps_segments[5])
        except ValueError: pass
        else:
            self.air_temperature = value
            parsed = True

        try:
            value = float(self.gps_segments[7])
        except ValueError: pass
        else:
            self.water_temperature = value
            parsed = True

        try:
            value = float(self.gps_segments[9])
        except ValueError: pass
        else:
            self.relative_humidity = value
            parsed = True

        try:
            value = float(self.gps_segments[11])
        except ValueError: pass
        else:
            self.dew_point = value
            parsed = True

        try:
            direction_true = float(self.gps_segments[13])
            direction_mag = float(self.gps_segments[15])
            speed_kts = float(self.gps_segments[17])
            parsed = True
        except ValueError: pass
        else:
            self.true_wind_angle = (direction_true, direction_mag)
            self.true_wind_speed = speed_kts * 1852 / 3600
            parsed = True

        return parsed

    def gpmtw(self):
        # $YDMTW,17.9,C*01
        try:
            value = float(self.gps_segments[1])
        except ValueError: pass
        else:
            self.water_temperature = value
            return True
        return

    def gpdbt(self):
        # $YDDBT,9.3,f,2.85,M,1.55,F*08
        try:
            value = float(self.gps_segments[3])
        except ValueError: pass
        else:
            self.depth_below_transducer = value
            return True
        return

    def gpdbs(self):
        # $YDDBS,10.9,f,3.35,M,1.83,F*3C
        try:
            value = float(self.gps_segments[3])
        except ValueError: pass
        else:
            self.depth_below_surface = value
            return True
        return

    def gpzda(self):
        # $YDZDA,055709.62,17,07,2022,-02,00*4A
        try:
            hours = int(self.gps_segments[5])
            mins= int(self.gps_segments[6])
        except ValueError: pass
        else:
            self.time_offset_minutes = hours * 60 + (mins * 1 if self.gps_segments[5][0] != "-" else -1)
            return True
        return

    supported_sentences = MicropyGPS.supported_sentences | {
        'VWR': gpvwr,
        'MDA': gpmda,
        'MTW': gpmtw,
        'DBT': gpdbt,
        'DBS': gpdbs,
        'ZDA': gpzda,
    }

