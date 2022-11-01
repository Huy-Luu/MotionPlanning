from __future__ import division

# For most use cases in this module, numpy is indistinguishable
# from math, except it also works on numpy arrays
try:
    import numpy as mathlib
    use_numpy = True
except ImportError:
    import math as mathlib
    use_numpy = False

class UTMmodule(object):
    __all__ = ['toLatlon', 'fromLatlon']

    K0 = 0.9996

    E = 0.00669438
    E2 = E * E
    E3 = E2 * E
    E_P2 = E / (1 - E)

    SQRT_E = mathlib.sqrt(1 - E)
    _E = (1 - SQRT_E) / (1 + SQRT_E)
    _E2 = _E * _E
    _E3 = _E2 * _E
    _E4 = _E3 * _E
    _E5 = _E4 * _E

    M1 = (1 - E / 4 - 3 * E2 / 64 - 5 * E3 / 256)
    M2 = (3 * E / 8 + 3 * E2 / 32 + 45 * E3 / 1024)
    M3 = (15 * E2 / 256 + 45 * E3 / 1024)
    M4 = (35 * E3 / 3072)

    P2 = (3 / 2 * _E - 27 / 32 * _E3 + 269 / 512 * _E5)
    P3 = (21 / 16 * _E2 - 55 / 32 * _E4)
    P4 = (151 / 96 * _E3 - 417 / 128 * _E5)
    P5 = (1097 / 512 * _E4)

    R = 6378137

    ZONE_LETTERS = "CDEFGHJKLMNPQRSTUVWXX"


    def inBounds(self, x, lower, upper, upper_strict=False):
        #print(x, lower, upper)
        if upper_strict and use_numpy:
            return lower <= mathlib.min(x) and mathlib.max(x) < upper
        elif upper_strict and not use_numpy:
            return lower <= x < upper
        elif use_numpy:
            return lower <= mathlib.min(x) and mathlib.max(x) <= upper
        return lower <= x <= upper


    def checkValidZone(self, zone_number, zone_letter):
        if not 1 <= zone_number <= 60:
            raise OutOfRangeError('zone number out of range (must be between 1 and 60)')

        if zone_letter:
            zone_letter = zone_letter.upper()

            if not 'C' <= zone_letter <= 'X' or zone_letter in ['I', 'O']:
                raise OutOfRangeError('zone letter out of range (must be between C and X)')


    def mixedSigns(self, x):
        return use_numpy and mathlib.min(x) < 0 and mathlib.max(x) >= 0


    def negative(self, x):
        if use_numpy:
            return mathlib.max(x) < 0
        return x < 0


    def modAngle(self, value):
        return (value + mathlib.pi) % (2 * mathlib.pi) - mathlib.pi


    def toLatLon(self, easting, northing, zone_number, zone_letter=None, northern=None, strict=True):
        if not zone_letter and northern is None:
            raise ValueError('either zone_letter or northern needs to be set')

        elif zone_letter and northern is not None:
            raise ValueError('set either zone_letter or northern, but not both')

        if strict:
            if not self.inBounds(easting, 100000, 1000000, upper_strict=True):
                raise self.OutOfRangeError('easting out of range (must be between 100,000 m and 999,999 m)')
            if not self.inBounds(northing, 0, 10000000):
                raise self.OutOfRangeError('northing out of range (must be between 0 m and 10,000,000 m)')
        
        self.checkValidZone(zone_number, zone_letter)
        
        if zone_letter:
            zone_letter = zone_letter.upper()
            northern = (zone_letter >= 'N')

        x = easting - 500000
        y = northing

        if not northern:
            y -= 10000000

        m = y / self.K0
        mu = m / (self.R * self.M1)

        p_rad = (mu +
                self.P2 * mathlib.sin(2 * mu) +
                self.P3 * mathlib.sin(4 * mu) +
                self.P4 * mathlib.sin(6 * mu) +
                self.P5 * mathlib.sin(8 * mu))

        p_sin = mathlib.sin(p_rad)
        p_sin2 = p_sin * p_sin

        p_cos = mathlib.cos(p_rad)

        p_tan = p_sin / p_cos
        p_tan2 = p_tan * p_tan
        p_tan4 = p_tan2 * p_tan2

        ep_sin = 1 - self.E * p_sin2
        ep_sin_sqrt = mathlib.sqrt(1 - self.E * p_sin2)

        n = self.R / ep_sin_sqrt
        r = (1 - self.E) / ep_sin

        c = self.E_P2 * p_cos**2
        c2 = c * c

        d = x / (n * self.K0)
        d2 = d * d
        d3 = d2 * d
        d4 = d3 * d
        d5 = d4 * d
        d6 = d5 * d

        latitude = (p_rad - (p_tan / r) *
                    (d2 / 2 -
                    d4 / 24 * (5 + 3 * p_tan2 + 10 * c - 4 * c2 - 9 * self.E_P2)) +
                    d6 / 720 * (61 + 90 * p_tan2 + 298 * c + 45 * p_tan4 - 252 * self.E_P2 - 3 * c2))

        longitude = (d -
                    d3 / 6 * (1 + 2 * p_tan2 + c) +
                    d5 / 120 * (5 - 2 * c + 28 * p_tan2 - 3 * c2 + 8 * self.E_P2 + 24 * p_tan4)) / p_cos

        longitude = self.modAngle(longitude + mathlib.radians(self.zoneNumberToCentralLongitude(zone_number)))

        return (mathlib.degrees(latitude),
                mathlib.degrees(longitude))


    def fromLatlon(self, latitude, longitude, force_zone_number=None, force_zone_letter=None):
        #print("In UTM module: " + str(latitude) + " " + str(longitude))
        if not self.inBounds(latitude, -80, 84):
            raise self.OutOfRangeError('latitude out of range (must be between 80 deg S and 84 deg N)')
        if not self.inBounds(longitude, -180, 180):
            raise self.OutOfRangeError('longitude out of range (must be between 180 deg W and 180 deg E)')
        if force_zone_number is not None:
            self.checkValidZone(force_zone_number, force_zone_letter)

        lat_rad = mathlib.radians(latitude)
        lat_sin = mathlib.sin(lat_rad)
        lat_cos = mathlib.cos(lat_rad)

        lat_tan = lat_sin / lat_cos
        lat_tan2 = lat_tan * lat_tan
        lat_tan4 = lat_tan2 * lat_tan2

        if force_zone_number is None:
            zone_number = self.latlonToZoneNumber(latitude, longitude)
        else:
            zone_number = force_zone_number

        if force_zone_letter is None:
            zone_letter = self.latitudeToZoneLetter(latitude)
        else:
            zone_letter = force_zone_letter

        lon_rad = mathlib.radians(longitude)
        central_lon = self.zoneNumberToCentralLongitude(zone_number)
        central_lon_rad = mathlib.radians(central_lon)

        n = self.R / mathlib.sqrt(1 - self.E * lat_sin**2)
        c = self.E_P2 * lat_cos**2

        a = lat_cos * self.modAngle(lon_rad - central_lon_rad)
        a2 = a * a
        a3 = a2 * a
        a4 = a3 * a
        a5 = a4 * a
        a6 = a5 * a

        m = self.R * (self.M1 * lat_rad -
                self.M2 * mathlib.sin(2 * lat_rad) +
                self.M3 * mathlib.sin(4 * lat_rad) -
                self.M4 * mathlib.sin(6 * lat_rad))

        easting = self.K0 * n * (a +
                            a3 / 6 * (1 - lat_tan2 + c) +
                            a5 / 120 * (5 - 18 * lat_tan2 + lat_tan4 + 72 * c - 58 * self.E_P2)) + 500000

        northing = self.K0 * (m + n * lat_tan * (a2 / 2 +
                                            a4 / 24 * (5 - lat_tan2 + 9 * c + 4 * c**2) +
                                            a6 / 720 * (61 - 58 * lat_tan2 + lat_tan4 + 600 * c - 330 * self.E_P2)))

        if self.mixedSigns(latitude):
            raise ValueError("latitudes must all have the same sign")
        elif self.negative(latitude):
            northing += 10000000

        return easting, northing


    def latitudeToZoneLetter(self, latitude):
        if use_numpy and isinstance(latitude, mathlib.ndarray):
            latitude = latitude.flat[0]

        if -80 <= latitude <= 84:
            return self.ZONE_LETTERS[int(latitude + 80) >> 3]
        else:
            return None


    def latlonToZoneNumber(self, latitude, longitude):
        if use_numpy:
            if isinstance(latitude, mathlib.ndarray):
                latitude = latitude.flat[0]
            if isinstance(longitude, mathlib.ndarray):
                longitude = longitude.flat[0]

        if 56 <= latitude < 64 and 3 <= longitude < 12:
            return 32

        if 72 <= latitude <= 84 and longitude >= 0:
            if longitude < 9:
                return 31
            elif longitude < 21:
                return 33
            elif longitude < 33:
                return 35
            elif longitude < 42:
                return 37

        return int((longitude + 180) / 6) + 1


    def zoneNumberToCentralLongitude(self, zone_number):
        return (zone_number - 1) * 6 - 180 + 3
