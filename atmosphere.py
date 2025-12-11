"""
Standard atmospheric model
Provides temperature, pressure, density, and speed of sound as functions of altitude
"""

import numpy as np

class Atmosphere:
    """
    International Standard Atmosphere (ISA) model
    Valid up to 11 km (troposphere)
    """
    
    # Sea level conditions
    T0 = 288.15  # K (15°C)
    P0 = 101325.0  # Pa
    rho0 = 1.225  # kg/m³
    g0 = 9.80665  # m/s²
    R = 287.05  # J/(kg·K) - specific gas constant for air
    gamma = 1.4  # Ratio of specific heats
    
    # Troposphere lapse rate
    L = -0.0065  # K/m (temperature decreases with altitude)
    
    def __init__(self):
        """Initialize atmosphere model"""
        pass
    
    def temperature(self, altitude):
        """Calculate temperature at given altitude"""
        if altitude < 11000:
            T = self.T0 + self.L * altitude
        else:
            T = self.T0 + self.L * 11000
        return T
    
    def pressure(self, altitude):
        """Calculate pressure at given altitude"""
        T = self.temperature(altitude)
        
        if altitude < 11000:
            P = self.P0 * (T / self.T0) ** (-self.g0 / (self.L * self.R))
        else:
            T_tropopause = self.T0 + self.L * 11000
            P_tropopause = self.P0 * (T_tropopause / self.T0) ** (-self.g0 / (self.L * self.R))
            P = P_tropopause * np.exp(-self.g0 * (altitude - 11000) / (self.R * T))
        
        return P
    
    def density(self, altitude):
        """Calculate air density at given altitude"""
        T = self.temperature(altitude)
        P = self.pressure(altitude)
        rho = P / (self.R * T)
        return rho
    
    def speed_of_sound(self, altitude):
        """Calculate speed of sound at given altitude"""
        T = self.temperature(altitude)
        a = np.sqrt(self.gamma * self.R * T)
        return a
    
    def dynamic_pressure(self, altitude, velocity):
        """Calculate dynamic pressure q = 0.5 * rho * V^2"""
        rho = self.density(altitude)
        q = 0.5 * rho * velocity**2
        return q
    
    def mach_number(self, altitude, velocity):
        """Calculate Mach number"""
        a = self.speed_of_sound(altitude)
        if a > 0:
            mach = velocity / a
        else:
            mach = 0.0
        return mach
    
    def get_properties(self, altitude):
        """Get all atmospheric properties at once"""
        return {
            'temperature': self.temperature(altitude),
            'pressure': self.pressure(altitude),
            'density': self.density(altitude),
            'speed_of_sound': self.speed_of_sound(altitude)
        }


class Wind:
    """Wind model with constant wind and turbulence"""
    
    def __init__(self, wind_north=0.0, wind_east=0.0, wind_up=0.0, 
                 turbulence_intensity=0.0):
        """Initialize wind model"""
        self.wind_north = wind_north
        self.wind_east = wind_east
        self.wind_up = wind_up
        self.turbulence_intensity = turbulence_intensity
        
        # For temporal correlation of turbulence
        self.prev_turbulence = np.zeros(3)
        self.turbulence_time_constant = 2.0  # seconds
    
    def get_wind_velocity(self, altitude, time, dt):
        """Calculate wind velocity at given altitude and time"""
        # Mean wind (could vary with altitude)
        wind_mean = np.array([self.wind_north, self.wind_east, self.wind_up])
        
        # Add turbulence (correlated random walk)
        if self.turbulence_intensity > 0:
            wind_magnitude = np.linalg.norm(wind_mean)
            
            # Random walk with exponential correlation
            alpha = dt / self.turbulence_time_constant
            random_component = np.random.randn(3)
            
            turbulence = (1 - alpha) * self.prev_turbulence + \
                        alpha * self.turbulence_intensity * wind_magnitude * random_component
            
            self.prev_turbulence = turbulence
        else:
            turbulence = np.zeros(3)
        
        wind_velocity = wind_mean + turbulence
        return wind_velocity
